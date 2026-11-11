#!/usr/bin/env python3
"""Accumulate existing usbmon CSV records; no USB/CAN sockets or ROS calls."""
import argparse, collections, csv, datetime, fcntl, json, os
from pathlib import Path
import shutil, signal, time

THRESHOLDS = (50, 150, 200, 300, 500)
FIELDS = ("bus","counter","urb_id","submit_ns","complete_ns","usb_duration_us",
          "submit_interval_us","complete_interval_us","status")
POST_SECONDS = 3.0
RAW_BUDGET = 4 * 1024**3
MIN_FREE = 4 * 1024**3

def iso(ns=None):
    value = time.time() if ns is None else ns / 1e9
    return datetime.datetime.fromtimestamp(value).astimezone().isoformat(timespec="microseconds")

def atomic_json(path, obj):
    tmp = path.with_name(path.name+".tmp")
    tmp.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+"\n")
    tmp.replace(path)

class Metric:
    def __init__(self, reference=0):
        self.reference=reference
        self.count=0
        self.invalid=0
        self.minimum=None
        self.maximum=None
        self.hist=collections.Counter()
        self.limits={t:{"positive":0,"negative":0,"absolute":0,"consecutive_now":0,"consecutive_max":0} for t in THRESHOLDS}
    def add(self, value, valid=True, reset_streak=False):
        if not valid:
            self.invalid+=1
            for c in self.limits.values():c["consecutive_now"]=0
            return
        value=round(float(value),3)
        self.count+=1
        self.minimum=value if self.minimum is None else min(self.minimum,value)
        self.maximum=value if self.maximum is None else max(self.maximum,value)
        self.hist[value]+=1
        delta=value-self.reference
        for threshold,c in self.limits.items():
            if reset_streak:c["consecutive_now"]=0
            above=delta>threshold
            below=delta < -threshold
            c["positive"]+=above
            c["negative"]+=below
            c["absolute"]+=above or below
            c["consecutive_now"]=c["consecutive_now"]+1 if above or below else 0
            c["consecutive_max"]=max(c["consecutive_max"],c["consecutive_now"])
    def percentile(self, q, absolute_bias=False):
        if not self.count:return None
        hist=collections.Counter()
        for value,count in self.hist.items():
            key=abs(value-self.reference) if absolute_bias else value
            hist[key]+=count
        target=max(1,int(self.count*q+.999999999))
        seen=0
        for value,count in sorted(hist.items()):
            seen+=count
            if seen>=target:return value
    def summary(self):
        return {
            "samples":self.count,"invalid_intervals":self.invalid,
            "min_us":self.minimum,"max_us":self.maximum,
            "p50_us":self.percentile(.5),"p99_us":self.percentile(.99),
            "min_signed_bias_us":None if self.minimum is None else round(self.minimum-self.reference,3),
            "max_signed_bias_us":None if self.maximum is None else round(self.maximum-self.reference,3),
            "max_negative_bias_us":None if self.minimum is None else round(min(0,self.minimum-self.reference),3),
            "max_positive_bias_us":None if self.maximum is None else round(max(0,self.maximum-self.reference),3),
            "max_absolute_bias_us":None if self.minimum is None else round(max(abs(self.minimum-self.reference),abs(self.maximum-self.reference)),3),
            "p99_absolute_bias_us":self.percentile(.99,True),
            "thresholds_us":{str(t):dict(c) for t,c in self.limits.items()}
        }

class Observer:
    def __init__(self, root):
        self.root=Path(root).resolve()
        self.out=self.root/"interval-statistics"
        self.out.mkdir(exist_ok=True)
        self.events_dir=self.out/"anomalies"
        self.events_dir.mkdir(exist_ok=True)
        self.lock=(self.out/"observer.lock").open("a")
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.offsets={}
        self.last_counter={}
        self.last_submit={}
        self.bus={i:{"rows":0,"counter_discontinuities":0,"submit":Metric(5000),"complete":Metric(5000),"usb":Metric(0)} for i in (0,1)}
        self.first_ns=None
        self.last_ns=None
        self.row_count=0
        self.max_segment=0
        self.gaps=[]
        self.pending=[]
        self.stop=False
        self.reason="running"
        self.sequence=0
        self.preserved_inodes=set()
        self.preserved_bytes=0
        self.disk_limited=False
        self.last_source_health={}
        self.event_count=0
        self.started=iso()
        self.source_at_start=json.loads((self.root/"status.json").read_text())
        self.interfaces=self.source_at_start.get('can_interfaces', ['can0', 'can1'])
        legacy_path=self.out/"legacy-maxima-at-install.json"
        if not legacy_path.exists():atomic_json(legacy_path,self.source_at_start)
        self.legacy=json.loads(legacy_path.read_text())
        self.event_log=self.out/"anomaly-events.jsonl"
        for p in self.events_dir.glob("event-*/event.json"):
            try:
                e=json.loads(p.read_text())
                self.sequence=max(self.sequence,int(e["sequence"]))
                self.event_count+=1
                if not e.get("complete",False):self.pending.append(e)
            except (OSError,ValueError,KeyError):pass
        for p in self.events_dir.glob("event-*/segment-*"):
            try:
                st=p.stat();key=(st.st_dev,st.st_ino)
                if key not in self.preserved_inodes:
                    self.preserved_inodes.add(key)
                    self.preserved_bytes+=max(st.st_size,4*1024**2 if p.suffix==".bin" else 256*1024)
            except OSError:pass
        # Existing full CSV is authoritative for restart/replay and source offsets.
        all_path=self.out/"sync-all.csv"
        if all_path.exists():
            with all_path.open() as f:
                for record in csv.DictReader(f):
                    try:
                        name=record["source_segment"]
                        end=int(record["source_end_offset"])
                        self.consume(record, replay=True)
                        self.offsets[name]=max(self.offsets.get(name,0),end)
                        self.max_segment=max(self.max_segment,int(name.split("-")[1].split(".")[0]))
                    except (ValueError,TypeError,KeyError):
                        self.reason="incomplete_saved_csv_row"
                        break
        self.all_file=all_path.open("a",buffering=65536,newline="")
        self.all_writer=csv.DictWriter(self.all_file,fieldnames=FIELDS+("source_segment","source_end_offset"))
        if all_path.stat().st_size==0:self.all_writer.writeheader()
        self.all_file.flush()
        (self.out/"pid").write_text(str(os.getpid())+"\n")
        (self.out/"FORMAT.txt").write_text(
            "Host usbmon timestamps for SYNC-containing URBs; these are not CAN-wire transmit or drive receive timestamps.\n"
            "Reference period: 5000 us. All threshold comparisons are strict >; negative means period < reference-threshold.\n"
            "Full interval statistics cover only rows retained in sync-all.csv from coverage_from onward.\n"
            "Older discarded healthy windows cannot be reconstructed; previous whole-capture maxima are kept separately.\n"
            "Raw anomaly triggers: |submit bias|>150us, |completion bias|>200us, or USB duration>300us;\n"
            "also 3 consecutive |bias|>150us, any EMCY, or capture loss/error. These are diagnostic thresholds, not FF05 firmware rules.\n"
            "Anomaly raw snapshots link the available rolling window (~65-70s) plus >=3s post-trigger where available.\n"
            "Five-second segment granularity can retain additional surrounding data. Source capture may end 3s after EMCY.\n"
            "Unique raw reservation budget 4GiB; minimum free disk 4GiB. No saved evidence is automatically deleted.\n")
    def consume(self, record, replay=False):
        b=int(record["bus"])
        if b not in self.bus:return
        row={k:record[k] for k in FIELDS}
        n=int(row["submit_ns"])
        counter=int(row["counter"])
        d=self.bus[b]
        discontinuity=b in self.last_counter and (counter-self.last_counter[b])%256!=1
        d["counter_discontinuities"]+=discontinuity
        self.last_counter[b]=counter
        self.last_submit[b]=n
        first=d["rows"]==0
        d["rows"]+=1
        self.row_count+=1
        self.first_ns=n if self.first_ns is None else min(self.first_ns,n)
        self.last_ns=n if self.last_ns is None else max(self.last_ns,n)
        submit=float(row["submit_interval_us"])
        complete=float(row["complete_interval_us"])
        duration=float(row["usb_duration_us"])
        d["submit"].add(submit,valid=submit>0,reset_streak=discontinuity)
        d["complete"].add(complete,valid=complete>0,reset_streak=discontinuity)
        d["usb"].add(duration,valid=duration>=0,reset_streak=discontinuity)
        if replay:return
        reasons=[]
        if submit>0 and abs(submit-5000)>150:reasons.append("submit_bias_gt_150us")
        if complete>0 and abs(complete-5000)>200:reasons.append("complete_bias_gt_200us")
        if duration>300:reasons.append("usb_duration_gt_300us")
        for metric in ("submit","complete"):
            if self.bus[b][metric].limits[150]["consecutive_now"]==3:
                reasons.append(metric+"_3_consecutive_abs_bias_gt_150us")
        if not first and (submit<=0 or complete<=0):reasons.append("nonpositive_interval")
        if discontinuity:reasons.append("sync_counter_discontinuity")
        if int(row["status"])!=0:reasons.append("usb_completion_error")
        if reasons:self.trigger(reasons,n,{"bus":self.interfaces[b],"row":row})
    def trigger(self,reasons,ns,detail):
        self.event_log.parent.mkdir(exist_ok=True)
        with self.event_log.open("a") as f:
            f.write(json.dumps({"host_ns":ns,"host_time":iso(ns),"reasons":reasons,"detail":detail})+"\n")
        # Adjacent long/short periods are grouped in the same evidence window.
        event=None
        if self.pending:
            last=self.pending[-1]
            if ns<=last["latest_trigger_ns"]+int(POST_SECONDS*1e9):
                event=last
        if event is None:
            self.sequence+=1
            event={"sequence":self.sequence,"first_trigger_ns":ns,"first_trigger_time":iso(ns),
                   "latest_trigger_ns":ns,"latest_trigger_time":iso(ns),"trigger_count":0,
                   "reasons":{},"first_detail":detail,"complete":False,"linked_files":[],
                   "raw_scope":"Available 65-70 second rolling window plus post-trigger segments",
                   "post_seconds_requested":POST_SECONDS}
            folder=self.events_dir/("event-%06d"%self.sequence)
            folder.mkdir()
            event["directory"]=str(folder)
            self.pending.append(event)
            self.event_count+=1
        event["latest_trigger_ns"]=max(event["latest_trigger_ns"],ns)
        event.pop("finalize_segment",None)
        event["latest_trigger_time"]=iso(event["latest_trigger_ns"])
        event["trigger_count"]+=1
        for reason in reasons:event["reasons"][reason]=event["reasons"].get(reason,0)+1
        atomic_json(Path(event["directory"])/"event.json",event)
    def link_snapshot(self,event):
        target=Path(event["directory"])
        try:free=shutil.disk_usage(self.root).free
        except OSError:free=MIN_FREE
        for source in sorted(self.root.glob("segment-*")):
            if source.suffix not in (".bin",".csv"):continue
            if "finalize_segment" in event:
                number=int(source.name.split("-")[1].split(".")[0])
                if number>event["finalize_segment"]:continue
            dest=target/source.name
            if dest.exists():continue
            try:
                st=source.stat();key=(st.st_dev,st.st_ino)
                reserve=max(st.st_size,4*1024**2) if source.suffix==".bin" else max(st.st_size,256*1024)
                if key not in self.preserved_inodes and (self.preserved_bytes+reserve>RAW_BUDGET or free<MIN_FREE):
                    self.disk_limited=True
                    event["preservation_limited"]=True
                    continue
                os.link(source,dest)
                event["linked_files"].append(source.name)
                if key not in self.preserved_inodes:
                    self.preserved_inodes.add(key);self.preserved_bytes+=reserve
            except FileNotFoundError:pass
        for name in ["session.json","status.json","FIRST_EMCY.json","emcy.jsonl"]:
            p=self.root/name
            if p.exists():
                try:(target/("source-"+name)).write_bytes(p.read_bytes())
                except OSError:pass
    def scan(self):
        paths=sorted(self.root.glob("segment-*.sync.csv"))
        ids=[int(p.name.split("-")[1].split(".")[0]) for p in paths]
        if self.max_segment and ids and min(ids)>self.max_segment+1:
            gap={"after_segment":self.max_segment,"next_available":min(ids),"detected_at":iso()}
            if not self.gaps or self.gaps[-1]!=gap:self.gaps.append(gap)
        for p in paths:
            try:
                with p.open("rb") as f:
                    offset=self.offsets.get(p.name,0)
                    f.seek(offset)
                    if offset==0:
                        header=f.readline()
                        if not header.endswith(b"\n"):continue
                        offset=f.tell()
                    while True:
                        start=f.tell();line=f.readline()
                        if not line:break
                        if not line.endswith(b"\n"):
                            f.seek(start);break
                        end=f.tell()
                        try:
                            parts=next(csv.reader([line.decode("ascii")]))
                            if len(parts)!=len(FIELDS):raise ValueError("column count")
                            record=dict(zip(FIELDS,parts))
                            self.consume(record)
                            record["source_segment"]=p.name
                            record["source_end_offset"]=end
                            self.all_writer.writerow(record)
                        except (ValueError,TypeError,KeyError) as e:
                            self.trigger(["source_csv_parse_error"],time.time_ns(),{"file":p.name,"offset":start,"error":str(e)})
                        offset=end
                    self.offsets[p.name]=offset
                self.max_segment=max(self.max_segment,int(p.name.split("-")[1].split(".")[0]))
            except FileNotFoundError:pass
        self.all_file.flush()
    def update_source(self):
        source=json.loads((self.root/"status.json").read_text())
        for field in ("dropped","truncated","completion_errors"):
            value=source.get(field,0)
            previous=self.last_source_health.get(field,0)
            if value>previous:self.trigger(["capture_"+field+"_increased"],time.time_ns(),{field:value,"previous":previous})
            self.last_source_health[field]=value
        fault=source.get("first_emcy")
        if fault and not self.last_source_health.get("first_emcy"):
            self.trigger(["first_emcy"],int(fault["host_ns"]),fault)
            self.last_source_health["first_emcy"]=True
        return source
    def finalize_events(self,source):
        now_ns=time.time_ns()
        complete=source.get("status")=="complete"
        for event in list(self.pending):
            post_ready=now_ns>=event["latest_trigger_ns"]+int(POST_SECONDS*1e9)
            current_segment=int(source["current_segment"].split("-")[1].split(".")[0])
            if post_ready and "finalize_segment" not in event:
                event["finalize_segment"]=current_segment
            self.link_snapshot(event)
            tail_closed="finalize_segment" in event and current_segment>event["finalize_segment"]
            if (post_ready and tail_closed) or complete:
                event["complete"]=True
                event["finalized_at"]=iso()
                event["source_complete_at_finalize"]=complete
                folder=Path(event["directory"])
                for p in self.root.glob("snapshot-*.log"):
                    try:(folder/p.name).write_bytes(p.read_bytes())
                    except OSError:pass
                if complete:
                    event["post_capture_note"]="Source recorder may end 3s after EMCY/controller exit."
                self.pending.remove(event)
            atomic_json(Path(event["directory"])/"event.json",event)
    def summary(self,source,final=False):
        return {
            "schema":1,"observer_pid":os.getpid(),"source_collector_pid":source["pid"],
            "source_controller_pid":source["controller_pid"],
            "status":"complete" if final else "monitoring","reason":self.reason,
            "updated_at":iso(),"observer_started_at":self.started,
            "measurement":"Adjacent same-bus SYNC-containing USB URB submit/complete timestamps, not CAN-wire TX or drive RX.",
            "reference_period_us":5000,"threshold_comparison":"strict >; counts include positive and negative separately",
            "thresholds_us":list(THRESHOLDS),
            "coverage_from":iso(self.first_ns) if self.first_ns is not None else None,
            "coverage_to":iso(self.last_ns) if self.last_ns is not None else None,
            "covered_seconds":None if self.first_ns is None else round((self.last_ns-self.first_ns)/1e9,3),
            "rows_retained_full_session":self.row_count,
            "full_csv":str(self.out/"sync-all.csv"),
            "coverage_note":"Statistics cover retained full CSV only. Earlier discarded healthy windows are unavailable.",
            "legacy_full_capture_maxima":self.legacy.get("sync"),
            "legacy_capture_started_at":self.legacy.get("started_at"),
            "buses":{self.interfaces[b]:{"rows":v["rows"],"counter_discontinuities":v["counter_discontinuities"],
                    "submit_interval":v["submit"].summary(),"complete_interval":v["complete"].summary(),
                    "usb_duration":v["usb"].summary()} for b,v in self.bus.items()},
            "source_health":{k:source.get(k) for k in ["status","dropped","truncated","completion_errors","first_emcy"]},
            "source_file_gaps":self.gaps,"anomaly_windows":self.event_count,
            "pending_anomaly_windows":len(self.pending),
            "raw_retention_budget_bytes":RAW_BUDGET,"raw_reserved_bytes":self.preserved_bytes,
            "raw_preservation_limited":self.disk_limited,
            "affinity":sorted(os.sched_getaffinity(0)),"nice":os.getpriority(os.PRIO_PROCESS,0),
            "can_or_usb_device_opened":False
        }
    def run(self):
        for sig in (signal.SIGTERM,signal.SIGINT):
            signal.signal(sig,lambda *_:setattr(self,"stop",True))
        missing=0
        source=self.source_at_start
        try:
            while not self.stop:
                self.scan()
                source=self.update_source()
                self.finalize_events(source)
                atomic_json(self.out/"statistics.json",self.summary(source))
                if source.get("status")=="complete":
                    self.reason="source_capture_complete";break
                if not Path("/proc/%d"%source["pid"]).exists():missing+=1
                else:missing=0
                if missing>=3:self.reason="source_capture_process_exited";break
                time.sleep(.75)
            if self.stop:self.reason="observer_stopped"
        except BaseException as e:
            self.reason="observer_error: "+repr(e)
            raise
        finally:
            self.scan()
            self.finalize_events(source)
            atomic_json(self.out/"statistics.json",self.summary(source,final=True))
            self.all_file.close()
            self.lock.close()

def main():
    p=argparse.ArgumentParser()
    p.add_argument("capture_directory")
    args=p.parse_args()
    Observer(args.capture_directory).run()

if __name__=="__main__":
    main()
