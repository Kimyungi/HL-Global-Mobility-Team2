"""Replay versioned dumps and causally join commands to original CAN samples.
Usage: python3 extract.py /path/to/workspace/drive_logs
Requires git, tar and g++; no ROS or running vehicle required.
"""
import argparse,bisect,csv,hashlib,io,json,math,struct,subprocess,tempfile
from pathlib import Path
from decimal import Decimal
from datetime import datetime,timezone,timedelta
HERE=Path(__file__).resolve().parent
REPO=Path(subprocess.check_output(['git','-C',str(HERE),'rev-parse','--show-toplevel'],text=True).strip())
VERSIONS={45:'0be273b01ef28d223b5b6643cfe16e62956123ad',46:'d92ef8b7ef685c2c8440c847e35dd21554bbc422'}
RUNS=['v2_20260920_160548_762545','v2_20260920_132033_650355']
KST=timezone(timedelta(hours=9))
MAX_AGE_NS=100_000_000

def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def read_csv(p):
 with p.open(newline='') as f:return list(csv.DictReader(f))

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('log_root',type=Path);args=parser.parse_args()
 summary=[]
 for name in RUNS:
  run=args.log_root/name;dump=run/'mgm_snapshots.bin'
  with dump.open('rb') as f:magic,version,record_size,param_size=struct.unpack('<IIII',f.read(16))
  assert magic==0x314D474D and version in VERSIONS
  assert (dump.stat().st_size-16-param_size)%record_size==0
  with tempfile.TemporaryDirectory(prefix='drive-signals-') as temp:
   temp=Path(temp);revision=VERSIONS[version]
   archive=subprocess.check_output(['git','-C',str(REPO),'archive',revision,'src/adas_mgm/core','src/adas_mgm/tools/dump_reader.hpp','src/adas_mgm/tools/dump_format.hpp'])
   subprocess.run(['tar','-x','-C',str(temp)],input=archive,check=True)
   source=temp/'src/adas_mgm'
   executable=temp/'replay'
   subprocess.run(['g++','-std=c++17','-O2','-I'+str(source),str(HERE/'replay_signals.cpp'),*[str(p) for p in sorted((source/'core').glob('*.cpp'))],'-o',str(executable)],check=True)
   replay=list(csv.DictReader(io.StringIO(subprocess.check_output([str(executable),str(dump)],text=True))))
  assert len(replay)==(dump.stat().st_size-16-param_size)//record_size
  transitions=read_csv(run/'transitions.csv');errors=[]
  keys=['top','navigation','avoidance','signal','safety','mission','mission_type','reference_source','speed_owner','last_mission_phase','last_mission_route_id','last_mission_fallback']
  for transition in transitions:
   row=replay[int(transition['tick'])]
   for key in keys:
    if row[key]!=transition[key]:errors.append([transition['tick'],key,row[key],transition[key]])
   if not math.isclose(float(row['v_ref']),float(transition['v_ref']),rel_tol=1e-5,abs_tol=1e-5):errors.append([transition['tick'],'v_ref',row['v_ref'],transition['v_ref']])
  assert not errors,errors[:10]
  times=[int(r['time_ns']) for r in replay]
  assert all(a<=b for a,b in zip(times,times[1:])), 'Snapshot clock rollback; manual alignment required'
  feedback=read_csv(run/'vehicle_vector.csv')
  ftimes=[int(Decimal(r['stamp_s'])*1_000_000_000) for r in feedback]
  assert all(a<=b for a,b in zip(ftimes,ftimes[1:])), 'CAN clock rollback'
  fields=['stamp_s','time_kst','elapsed_s','v_ref','v_act','str_ref','str_act','x','y','state','gps_position_valid','feedback_x','feedback_y','mgm_tick','mgm_age_ms','matched','feedback_gap_s']
  missing=0;invalid_gps=0
  out=HERE/(name+'.csv')
  with out.open('w',newline='') as f:
   writer=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");writer.writeheader()
   for i,(v,t) in enumerate(zip(feedback,ftimes)):
    idx=bisect.bisect_right(times,t)-1
    age=t-times[idx] if idx>=0 else None
    valid=age is not None and 0<=age<=MAX_AGE_NS
    r=replay[idx] if valid else None
    missing+=not valid
    invalid_gps+=bool(r and r['gps_position_valid']!='1')
    writer.writerow(dict(stamp_s=v['stamp_s'],time_kst=datetime.fromtimestamp(t/1e9,KST).isoformat(timespec='milliseconds'),elapsed_s=f'{(t-ftimes[0])/1e9:.3f}',v_ref=r['v_ref'] if r else '',v_act=v['v'],str_ref=v['str_ref'],str_act=v['str'],x=r['gps_x'] if r and r['gps_position_valid']=='1' else '',y=r['gps_y'] if r and r['gps_position_valid']=='1' else '',state=r['state'] if r else '',gps_position_valid=r['gps_position_valid'] if r else '',feedback_x=v['x'],feedback_y=v['y'],mgm_tick=r['tick'] if r else '',mgm_age_ms=f'{age/1e6:.6f}' if age is not None else '',matched=int(valid),feedback_gap_s=f'{(t-ftimes[i-1])/1e9:.3f}' if i else ''))
  # Each exported row preserves one original CAN sample, without rounding anew.
  exported=read_csv(out);assert len(exported)==len(feedback)
  for a,b in zip(exported,feedback):
   for dst,src in [('stamp_s','stamp_s'),('v_act','v'),('str_ref','str_ref'),('str_act','str'),('feedback_x','x'),('feedback_y','y')]:assert a[dst]==b[src]
  info=dict(run=name,dump_version=version,replay_revision=revision,snapshot_rows=len(replay),can_rows=len(feedback),output_rows=len(exported),unmatched_rows=missing,invalid_gps_rows=invalid_gps,first_kst=exported[0]['time_kst'],last_kst=exported[-1]['time_kst'],max_feedback_gap_s=max((b-a)/1e9 for a,b in zip(ftimes,ftimes[1:])),max_snapshot_gap_s=max((b-a)/1e9 for a,b in zip(times,times[1:])),snapshot_last_kst=datetime.fromtimestamp(times[-1]/1e9,KST).isoformat(),transition_rows_verified=len(transitions),source_sha256={n:sha(run/n) for n in ['mgm_snapshots.bin','vehicle_vector.csv','transitions.csv','route_selected.yaml']},output_sha256=sha(out))
  summary.append(info);print(json.dumps(info,ensure_ascii=False),flush=True)
 (HERE/'manifest.json').write_text(json.dumps(dict(alignment='latest preceding MGM event_time_ns; maximum age 100ms; no interpolation',coordinate_source='GPS local metric pose (invalid poses blank); original CAN coordinates in feedback_x/feedback_y',runs=summary),ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
