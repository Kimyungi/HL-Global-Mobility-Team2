import csv,json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
out=Path(__file__).resolve().parent;run=out.parents[1]/'drive_logs/v2_20260914_224009_892949'
def read(path):
 with path.open() as f:return list(csv.DictReader(f))
def numeric(path):return [{k:float(v) for k,v in r.items()} for r in read(path)]
a=numeric(out/'inputs.csv');b=numeric(out/'replay.csv');d=[dict(x,**{k:v for k,v in y.items() if k!='tick'},stamp_s=x['event_time_ns']/1e9) for x,y in zip(a,b)]
fmt=lambda s:datetime.fromtimestamp(s,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
tr=numeric(run/'transitions.csv')
assert all(all(r[c]==b[int(r['tick'])][c] for c in ['top','navigation','avoidance','safety','ref_valid']) for r in tr)
print('6466 records; transition states match; max v_ref delta',max(abs(r['v_ref']-b[int(r['tick'])]['v_ref']) for r in tr))
for t in [3703,3707,3720,3740,3757,3766,3767,3866,4865,5027]:
 r=d[t];print(t,fmt(r['stamp_s']),'v',r['vehicle_speed'],'cmd',r['v_ref'],'avoid_n',r['avoid_path_n'],'provider',[round(r[k],5) for k in ['avoid_x','avoid_y','avoid_yaw','avoid_k']],'wire',[round(r[k],5) for k in ['x0','y0','yaw0','k0']],'age',r['avoid_age_s'])
valid=[r for r in d if r['avoidance']==1 and r['avoid_path_n']==1];print('valid span',len(valid),fmt(valid[0]['stamp_s']),fmt(valid[-1]['stamp_s']),'vcmdmax',max(r['v_ref'] for r in valid))
upd=[r for r in d[3695:3901] if r['avoid_updated']==1];print('updates',[(fmt(r['stamp_s']),int(r['avoid_path_n']),round(r['avoid_age_s'],3)) for r in upd]);print('max update gap',max(y['stamp_s']-x['stamp_s'] for x,y in zip(upd,upd[1:])))
v=numeric(run/'vehicle_vector.csv');g=read(run/'lateral.csv')
for t in [1789393247.297,1789393247.780,1789393248.873]:
 r=min(v,key=lambda r:abs(r['stamp_s']-t));print('vehicle',fmt(t),r)
vv=[r for r in v if 1789393247.297<=r['stamp_s']<=1789393247.780];print('validwindow VV', {k:(min(r[k] for r in vv),max(r[k] for r in vv)) for k in ['v','str','str_ref','yaw']})
gg=[r for r in g if 1789393247<=float(r['stamp_s'])<=1789393249]; print('GPS',len(gg),set(r['quality'] for r in gg),set(r['heading_src'] for r in gg),'maxfixage',max(float(r['fix_age_s']) for r in gg))
start=d[3703];stop=next(r for r in d[3704:] if abs(r['vehicle_speed'])<.05);seg=d[3703:int(stop['tick'])+1];print('stopped',fmt(stop['stamp_s']),'delay',stop['stamp_s']-start['stamp_s'],'forward_integral',sum(max(0,y['vehicle_speed'])*(y['stamp_s']-x['stamp_s']) for x,y in zip(seg,seg[1:])))
with (out/'entry_detail.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=d[0].keys());w.writeheader();w.writerows(d[3695:3901])
