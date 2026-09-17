import sqlite3,csv,json,math,bisect,collections
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
summaries=json.loads((OUT/'summary.json').read_text());result=[]
for ss in summaries:
 name=ss['name'];run=ROOT/'drive_logs'/name
 d=list(csv.DictReader((OUT/(name+'_decoded.csv')).open()));dt=[float(x['t']) for x in d];lo=next(float(x['t']) for x in d if x['top']=='1');hi=ss['manual_t']
 con=sqlite3.connect(f'file:{run}/rosbag/rosbag_0.db3?mode=ro',uri=True);topics={name:(id,typ) for id,name,typ in con.execute('select id,name,type from topics')}
 item={'run':name,'topics':{},'logs':[]}
 for topic in ['/perception/gps_path','/perception/avoid','/adas/target_ref','/rosout']:
  tid,typ=topics[topic];cls=get_message(typ);counts=collections.Counter();ages=[];mismatch=0
  for stamp,data in con.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<=? order by timestamp',(tid,int(lo*1e9),int(hi*1e9))):
   m=deserialize_message(data,cls);counts['messages']+=1
   if topic=='/perception/avoid':
    counts['with_points' if m.points else 'empty']+=1
    counts['detected' if m.obstacle_detected else 'clear']+=1
    if m.points:ages.append((stamp/1e9)-(m.reference_stamp.sec+m.reference_stamp.nanosec/1e9))
   elif topic=='/perception/gps_path':
    counts['fix_'+str(m.fix_quality)]+=1;counts['heading_'+str(m.heading_source)]+=1
   elif topic=='/adas/target_ref':
    t=m.header.stamp.sec+m.header.stamp.nanosec/1e9;i=bisect.bisect_left(dt,t);i=min(range(max(0,i-1),min(len(dt),i+2)),key=lambda k:abs(dt[k]-t));r=d[i]
    if abs(dt[i]-t)>.002:counts['unaligned']+=1;continue
    counts['compared']+=1
    if abs(float(r['v_ref'])-m.v_ref)>1e-6:counts['speed_mismatch']+=1
    if m.ref_points and any(abs(float(r['target_point_'+k])-getattr(m.ref_points[0],k))>1e-5 for k in ['x','y','yaw','curvature']):counts['geometry_mismatch']+=1
   else:
    if m.name in ('stack_avoid_node','stack_gps_node') and ('planner' in m.msg or 'compute' in m.msg or 'GPS' in m.msg or '헤딩' in m.msg):item['logs'].append({'t':stamp/1e9,'node':m.name,'msg':m.msg})
  item['topics'][topic]=dict(counts)
  if ages:
   ages.sort();item['avoid_pub_reference_age_s']={'min':min(ages),'median':ages[len(ages)//2],'max':max(ages),'over_0.2':sum(a>.2 for a in ages),'n':len(ages)}
 result.append(item)
(OUT/'bag_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='logs'} for r in result],ensure_ascii=False,indent=2))
