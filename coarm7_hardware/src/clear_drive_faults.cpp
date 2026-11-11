#include "coarm7_hardware/core.hpp"
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string_view>
#include <fcntl.h>
#include <sys/file.h>

int main(int argc, char **argv) {
  if (argc != 3 || std::string_view(argv[1]) != "--confirm-exclusive-can") {
    std::cerr << "Usage: clear_drive_faults --confirm-exclusive-can /path/to/hardware_reviewed.yaml\n"
              << "Exit all CAN control programs before confirming exclusive access.\n";
    return 2;
  }
  try {
    const auto config_path = std::filesystem::canonical(argv[2]);
    const auto config = coarm7_hardware::Config::load(config_path.string());
    // Same lock as Run Service. The process keeps it until the SDK closes CAN.
    const int lock = ::open(config_path.parent_path().c_str(),O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (lock < 0) {
      std::cerr << "Cannot open recovery directory lock: " << std::strerror(errno) << '\n';
      return 1;
    }
    if (::flock(lock,LOCK_EX | LOCK_NB) != 0) {
      std::cerr << "Cannot lock recovery directory (exit any active Run Service): "
                << std::strerror(errno) << '\n';
      return 1;
    }
    if (!coarm7_hardware::clear_sdk_drive_faults(config)) return 1;
    std::cout << "All drive fault codes are zero and fresh stationary disabled/ready feedback was verified. "
                 "Run Service separately to enable.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Drive fault recovery failed: " << error.what() << '\n';
    return 1;
  }
}
