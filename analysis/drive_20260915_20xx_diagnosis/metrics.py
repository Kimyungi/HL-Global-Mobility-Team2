import csv,json,math,bisect,collections
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
data=json.loads((OUT/'data.json').read_text());out=[]
def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi
def distance(p,a,b):
 dx=b[0]-a[0];dy=b[1]-a[1];v=((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy);v=max(0,min(1,v));return math.hypot(p[0]-a[0]-v*dx,p[1]-a[1]-v*dy)
for run in data:
 ss=run['summary'];rows=[{k:float(v) for k,v in x.items()} for x in csv.DictReader((OUT/(ss['name']+'_decoded.csv')).open())];cut=ss['manual_t'];act=[r for r in rows if r['top']==1 and r['t']<cut]
 avoid=[r for r in act if r['source']==2];blocked=[r for r in avoid if r['blocked']];reasons=collections.Counter('empty' if r['avoid_n']==0 else 'stale' if r['avoid_age']>r['avoid_timeout'] else 'other' for r in blocked)
 toggles=sum(a['v_ref']!=b['v_ref'] for a,b in zip(act,act[1:]) if b['t']-a['t']<.1)
 fs=[f for f in run['frames'] if f['top']==1 and not f['manual']];polys=list(run['routes'].values());cross=[min(distance(f['xy'],a,b) for ps in polys for a,b in zip(ps,ps[1:])) for f in fs]
 residual=[];jumps=[];headjumps=[]
 for i,f in enumerate(fs):
  if i and f['t']-fs[i-1]['t']<.15:
   jumps.append(math.dist(f['xy'],fs[i-1]['xy']));headjumps.append(abs(math.degrees(wrap(f['yaw']-fs[i-1]['yaw']))))
  if i+5<len(fs):
   b=fs[i+5];dt=b['t']-f['t'];dist=math.dist(b['xy'],f['xy'])
   if .4<dt<.7 and min(f['v'],b['v'])>.5 and dist>.25:
    cog=math.atan2(b['xy'][1]-f['xy'][1],b['xy'][0]-f['xy'][0]);yaw=f['yaw']+wrap(b['yaw']-f['yaw'])/2;residual.append(abs(math.degrees(wrap(cog-yaw))))
 residual.sort();metrics={'run':ss['name'],'avoid_ticks':len(avoid),'blocked_causes_seconds':{k:round(v*.01,2) for k,v in reasons.items()},'command_speed_changes':toggles,'true_distance_to_either_csv_max':round(max(cross),3),'gps_step_max_m':round(max(jumps),3),'heading_step_max_deg':round(max(headjumps),2),'motion_heading_residual_degrees':{'n':len(residual),'median':round(residual[len(residual)//2],2),'p95':round(residual[min(len(residual)-1,int(len(residual)*.95))],2),'max':round(max(residual),2)},'command_ref_beyond_endpoint_samples':[{'t':r['t'],'x':r['gps_point_x'],'y':r['gps_point_y'],'v':r['v']} for r in act if r['source']==1 and r['gps_point_x']<0 and r['v_ref']>0][::10]}
 out.append(metrics)
(OUT/'metrics.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
