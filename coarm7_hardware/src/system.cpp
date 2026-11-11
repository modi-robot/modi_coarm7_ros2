#include "coarm7_hardware/system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <fstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <urdf_parser/urdf_parser.h>

namespace coarm7_hardware {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
CoArm7System::~CoArm7System() { if(engine_) { engine_->deactivate(); engine_->cleanup(); } }
CallbackReturn CoArm7System::on_init(const hardware_interface::HardwareInfo &info) {
  if(SystemInterface::on_init(info)!=CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    if (info.hardware_parameters.count("effective_limits_file")) {
      effective_limits_file_=info.hardware_parameters.at("effective_limits_file");
      if (!std::filesystem::path(effective_limits_file_).is_absolute() || std::filesystem::exists(effective_limits_file_))
        throw std::invalid_argument("effective_limits_file must be a fresh absolute session path");
    }
    auto c=Config::load(info.hardware_parameters.at("config_file"));
    const auto model=urdf::parseURDF(info.original_xml);
    if (!model) throw std::invalid_argument("valid original URDF is required for arm position bounds");
    for (auto &axis:c.axes) {
      const auto joint=model->getJoint(axis.name);
      if (!joint || joint->type!=urdf::Joint::REVOLUTE || !joint->limits ||
          !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
          joint->limits->lower>=joint->limits->upper)
        throw std::invalid_argument("missing or invalid URDF position bounds: "+axis.name);
      axis.lower=joint->limits->lower;
      axis.upper=joint->limits->upper;
    }
    c.validate();
    for(std::size_t i=0;i<kArms;++i) names_[i]=c.axes[i].name;
    for(std::size_t i=0;i<kGrippers;++i) names_[kArms+i]=c.grippers[i].name;
    engine_=std::make_unique<Engine>(c,make_transport(c));
    std::array<bool,kCommands> seen{};
    for(const auto &j:info.joints) {
      if(j.state_interfaces.size()<2 || j.state_interfaces[0].name!="position" || j.state_interfaces[1].name!="velocity")
        throw std::invalid_argument("each joint requires position, velocity state interfaces in that order");
      const auto m=j.parameters.find("mimic");
      const auto name=m==j.parameters.end()?j.name:m->second;
      const auto it=std::find(names_.begin(),names_.end(),name);
      if(it==names_.end()) throw std::invalid_argument("unknown joint/mimic source: "+name);
      const auto index=std::distance(names_.begin(),it);
      double multiplier=1,offset=0;
      if(m!=j.parameters.end()) {
        if(j.state_interfaces.size()!=2 || !j.command_interfaces.empty() || index<static_cast<int>(kArms)) throw std::invalid_argument("invalid mimic interface");
        if(j.parameters.count("multiplier")) multiplier=std::stod(j.parameters.at("multiplier"));
        if(j.parameters.count("offset")) offset=std::stod(j.parameters.at("offset"));
        if(!std::isfinite(multiplier)||!std::isfinite(offset)) throw std::invalid_argument("nonfinite mimic mapping");
      } else {
        const bool grip=index>=static_cast<int>(kArms);
        if (j.state_interfaces.size()!=(grip?4u:2u) ||
            (grip && (j.state_interfaces[2].name!="feedback_stamp" || j.state_interfaces[3].name!="stop_ack")))
          throw std::invalid_argument("invalid feedback interfaces");
        if(seen[index] || j.command_interfaces.size()!=(grip?3u:1u) || j.command_interfaces[0].name!="position" ||
           (grip && (j.command_interfaces[1].name!="goal_id" || j.command_interfaces[2].name!="stop_id")))
          throw std::invalid_argument("expected exactly one position command per independent joint");
        seen[index]=true;
      }
      indices_.push_back(index); multipliers_.push_back(multiplier); offsets_.push_back(offset);
    }
    if(!std::all_of(seen.begin(),seen.end(),[](bool b){return b;})) throw std::invalid_argument("missing independent joint");
    state_position_.assign(info.joints.size(),0); state_velocity_.assign(info.joints.size(),0);
    return CallbackReturn::SUCCESS;
  } catch(const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"),"Initialization rejected: %s",e.what());
    return CallbackReturn::ERROR;
  }
}
CallbackReturn CoArm7System::on_configure(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  fault_reported_=false;
  try {
    configured_=engine_&&engine_->configure();
    if (configured_) export_effective_limits();
  }
  catch(const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"),"Configuration failed: %s",e.what());
    if(engine_) engine_->cleanup();
    configured_=false;
  }
  if (!configured_ && engine_) {
    const auto detail=engine_->fault_detail();
    if (!detail.empty())
      RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"),"Startup feedback rejected: %s",detail.c_str());
  }
  return configured_?CallbackReturn::SUCCESS:CallbackReturn::ERROR;
}
CallbackReturn CoArm7System::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  claims_.fill(false);
  if (!engine_) return CallbackReturn::ERROR;
  if (engine_->activate(monotonic_now_ns())) return CallbackReturn::SUCCESS;
  const auto detail=engine_->fault_detail();
  if (!detail.empty())
    RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"),"Activation rejected: %s",detail.c_str());
  return CallbackReturn::ERROR;
}
CallbackReturn CoArm7System::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_); claims_.fill(false);
  return !engine_||engine_->deactivate()?CallbackReturn::SUCCESS:CallbackReturn::ERROR;
}
CallbackReturn CoArm7System::on_cleanup(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  bool stopped=true;
  if(engine_) { stopped=engine_->deactivate(); engine_->cleanup(); }
  configured_=false; claims_.fill(false);
  if (!effective_limits_file_.empty()) {
    std::error_code ignored; std::filesystem::remove(effective_limits_file_,ignored);
  }
  return stopped?CallbackReturn::SUCCESS:CallbackReturn::ERROR;
}
CallbackReturn CoArm7System::on_shutdown(const rclcpp_lifecycle::State &s) { return on_cleanup(s); }
CallbackReturn CoArm7System::on_error(const rclcpp_lifecycle::State &s) { return on_cleanup(s); }
std::vector<hardware_interface::StateInterface> CoArm7System::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for(std::size_t i=0;i<info_.joints.size();++i) {
    result.emplace_back(info_.joints[i].name,"position",&state_position_[i]);
    result.emplace_back(info_.joints[i].name,"velocity",&state_velocity_[i]);
  }
  for(std::size_t i=0;i<kGrippers;++i) {
    result.emplace_back(names_[kArms+i],"feedback_stamp",&engine_->gripper_feedback_stamps()[i]);
    result.emplace_back(names_[kArms+i],"stop_ack",&engine_->gripper_stop_acks()[i]);
  }
  return result;
}
std::vector<hardware_interface::CommandInterface> CoArm7System::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for(std::size_t i=0;i<kCommands;++i) result.emplace_back(names_[i],"position",&engine_->commands()[i]);
  for(std::size_t i=0;i<kGrippers;++i) {
    result.emplace_back(names_[kArms+i],"goal_id",&engine_->gripper_goal_ids()[i]);
    result.emplace_back(names_[kArms+i],"stop_id",&engine_->gripper_stop_ids()[i]);
  }
  return result;
}
return_type CoArm7System::read(const rclcpp::Time &,const rclcpp::Duration &) {
  std::unique_lock<std::mutex> lock(lifecycle_mutex_,std::try_to_lock);
  // Lifecycle SDK startup/teardown owns the bus while this lock is held.
  if(!lock.owns_lock()||!configured_) return return_type::OK;
  if(!engine_->read(monotonic_now_ns())) {
    report_fault_once("read");
    return return_type::ERROR;
  }
  for(std::size_t i=0;i<indices_.size();++i) {
    state_position_[i]=engine_->positions()[indices_[i]]*multipliers_[i]+offsets_[i];
    state_velocity_[i]=engine_->velocities()[indices_[i]]*multipliers_[i];
  }
  return return_type::OK;
}
return_type CoArm7System::write(const rclcpp::Time &,const rclcpp::Duration &) {
  std::unique_lock<std::mutex> lock(lifecycle_mutex_,std::try_to_lock);
  if(!lock.owns_lock()||!configured_) return return_type::OK;
  if (engine_->write()) return return_type::OK;
  report_fault_once("write");
  return return_type::ERROR;
}
void CoArm7System::report_fault_once(const char *phase) {
  if (fault_reported_ || !engine_ || engine_->fault()==Fault::none) return;
  fault_reported_=true;
  const auto detail=engine_->fault_detail();
  RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"),
    "First hardware rejection phase=%s fault=%d: %s; stop_submitted_at_check=%s (not physical acknowledgement)",
    phase,static_cast<int>(engine_->fault()),detail.empty()?"no joint detail":detail.c_str(),
    engine_->stop_delivery_ok()?"true":"false");
}
void CoArm7System::export_effective_limits() {
  if (effective_limits_file_.empty()) return;
  const auto &c=engine_->config();
  YAML::Emitter out; out.SetDoublePrecision(17);
  out << YAML::BeginMap << YAML::Key << "schema" << YAML::Value << 1
      << YAML::Key << "config_file" << YAML::Value << info_.hardware_parameters.at("config_file")
      << YAML::Key << "component" << YAML::Value << info_.name
      << YAML::Key << "coordinate_system" << YAML::Value << "ros_joint_radians"
      << YAML::Key << "position_limits_source" << YAML::Value << "urdf"
      << YAML::Key << "axes" << YAML::Value << YAML::BeginMap;
  for (const auto &axis:c.axes) {
    out << YAML::Key << axis.name << YAML::Value << YAML::BeginMap
        << YAML::Key << "lower" << YAML::Value << axis.lower
        << YAML::Key << "upper" << YAML::Value << axis.upper << YAML::EndMap;
  }
  out << YAML::EndMap << YAML::EndMap;
  const auto temporary=effective_limits_file_+".tmp";
  try {
    std::ofstream file(temporary,std::ios::trunc);
    file.exceptions(std::ios::badbit|std::ios::failbit);
    file << out.c_str() << '\n'; file.close();
    std::filesystem::rename(temporary,effective_limits_file_);
  } catch (...) {
    std::error_code ignored; std::filesystem::remove(temporary,ignored); throw;
  }
  RCLCPP_INFO(rclcpp::get_logger("coarm7_hardware"),"Effective planning limits exported after configure: %s",effective_limits_file_.c_str());
}
bool CoArm7System::next_claims(const std::vector<std::string> &start,const std::vector<std::string> &stop,std::array<bool,kCommands> &next) const {
  next=claims_;
  // Endpoint position/goal/stop interfaces must be claimed as one unit.
  for (const auto *keys : {&start,&stop}) for(std::size_t i=kArms;i<kCommands;++i) {
    int count=0;
    for(const auto &key:*keys) if(key.rfind(names_[i]+"/",0)==0) ++count;
    if(count!=0 && count!=3) return false;
  }
  auto apply=[&](const auto &keys,bool value) {
    for(const auto &key:keys) for(std::size_t i=0;i<kCommands;++i) {
      if(key.rfind(names_[i]+"/",0)!=0) continue;
      if(key!=names_[i]+"/position") {
        if(i<kArms || (key!=names_[i]+"/goal_id" && key!=names_[i]+"/stop_id")) return false;
        continue; // Position interface alone determines motion ownership.
      }
      if(value && i>=kArms && !engine_->config().gripper_commands_enabled()) {
        RCLCPP_ERROR(rclcpp::get_logger("coarm7_hardware"), "Cannot claim read-only gripper: %s", names_[i].c_str());
        return false;
      }
      next[i]=value;
    }
    return true;
  };
  return apply(stop,false)&&apply(start,true);
}
return_type CoArm7System::prepare_command_mode_switch(const std::vector<std::string> &start,const std::vector<std::string> &stop) {
  std::array<bool,kCommands> next{};
  return next_claims(start,stop,next)?return_type::OK:return_type::ERROR;
}
return_type CoArm7System::perform_command_mode_switch(const std::vector<std::string> &start,const std::vector<std::string> &stop) {
  std::array<bool,kCommands> next{};
  if(!next_claims(start,stop,next)||!engine_->set_claims(next)) return return_type::ERROR;
  claims_=next; return return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(coarm7_hardware::CoArm7System, hardware_interface::SystemInterface)
