import csv,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/stack_gps'),str(ROOT/'src/stack_parking')]
from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_parking.t_parking_sequence import load_course
D=ROOT/'src/stack_gps/waypoints'
plan=RoutePlan(D/'halla_route_sequence.yaml','01','07')
origin=plan.files[0].points[0]
result={'commit':'2991216','sequence':[r.id for r in plan.files],'routes':[],'junctions':[]}
for i in range(1,8):
 p=D/f'halla_0919_path_{i:02}.csv'
 rows=list(csv.DictReader(p.open()));lat=np.array([float(r['lat']) for r in rows]);lon=np.array([float(r['lon']) for r in rows]);yaw=np.unwrap([float(r['yaw_rad']) for r in rows])
 xy=np.c_[(lon-origin[1])*111320*np.cos(np.radians(origin[0])),(lat-origin[0])*111320]
 ds=np.linalg.norm(np.diff(xy,axis=0),axis=1);s=np.r_[0,np.cumsum(ds)]
 k=np.gradient(yaw,s);bad=np.flatnonzero(abs(k)>1/1.15)
 result['routes'].append(dict(id=f'{i:02}',points=len(rows),length=float(s[-1]),min_spacing=float(min(ds)),max_spacing=float(max(ds)),min_radius=float(1/max(abs(k))),bad_indices=bad.tolist(),max_curvature_index=int(np.argmax(abs(k))),events=[{'idx':n,'state':int(r['state']),'zone':int(r['zone_id'])} for n,r in enumerate(rows) if int(r['state'])!=0]))
for a,b in zip(plan.files,plan.files[1:]):
 ra=list(csv.DictReader(a.csv.open()));rb=list(csv.DictReader(b.csv.open()))
 delta=(float(rb[0]['yaw_rad'])-float(ra[-1]['yaw_rad'])+math.pi)%(2*math.pi)-math.pi
 result['junctions'].append(dict(routes=[a.id,b.id],gap_m=math.hypot((a.points[-1][0]-b.points[0][0])*111320,(a.points[-1][1]-b.points[0][1])*111320*math.cos(math.radians(origin[0]))),yaw_delta_deg=math.degrees(delta)))
cs,approach,station=load_course(plan.files[0].csv,plan.files[1].csv,[ROOT/f'src/stack_parking/config/parking_ref_{i:02}.csv' for i in (1,2)])
result['parking']=[dict(name=c.name,min_radius=c.minimum_radius,max_required_steer_deg=math.degrees(math.atan(.595/c.minimum_radius)),start_yaw_delta_deg=math.degrees((c.path[0].yaw-approach[-1].yaw+math.pi)%(2*math.pi)-math.pi),length=float(c.s[-1])) for c in cs]
Path(__file__).with_name('metrics.json').write_text(json.dumps(result,indent=2))
print(json.dumps({**result,'routes':[{k:v for k,v in r.items() if k!='events'} for r in result['routes']]},indent=2))
import yaml
from stack_gps.path_engine import load_waypoints_csv,avoidance_marker_range
from stack_gps.zones import ZoneMap,turn_zone_map
engines={}
result['runtime']=[]
for f in plan.files:
 e=PathEngine(f.points,n_points=1,station_tracking=True);engines[f.id]=e
 cfg=yaml.safe_load(f.zones.read_text())
 for pt in cfg['parking_points']:
  a,b,d=e.range_from_latlon(pt['lat'],pt['lon'],1.)
  (e.parking_ranges if pt['mode']=='perpendicular' else e.parallel_parking_ranges).append((a,b))
 z=turn_zone_map(f.zones,e,5.,ZoneMap.from_engine(e))
 bad=np.flatnonzero(np.abs(e.curvature)>1/1.15)
 result['runtime'].append(dict(id=f.id,min_radius=1/max(abs(v) for v in e.curvature),over_limit_indices=bad.tolist(),turn_ranges=e.gps_only_ranges,t_parking=e.parking_ranges,parallel=e.parallel_parking_ranges,avoid=avoidance_marker_range(f.csv)))
result['runtime_junctions']=[dict(routes=[a.id,b.id],yaw_delta_deg=math.degrees((engines[b.id].yaw[0]-engines[a.id].yaw[-1]+math.pi)%(2*math.pi)-math.pi)) for a,b in zip(plan.files,plan.files[1:])]
Path(__file__).with_name('metrics.json').write_text(json.dumps(result,indent=2))
print('RUNTIME',json.dumps(result['runtime'],indent=2));print('JUNCTIONS',result['runtime_junctions'])
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,(ax,bx)=plt.subplots(1,2,figsize=(13,7))
for f in plan.files:
 pts=np.array([engines[plan.files[0].id].to_enu(*p) for p in f.points]);ax.plot(*pts.T,label=f.id)
 k=np.abs(engines[f.id].curvature);ids=np.flatnonzero(k>1/1.15)
 if len(ids):ax.scatter(*pts[ids].T,c='red',s=30)
 bx.plot(engines[f.id].curvature,label=f.id)
for c in cs:ax.plot([p.x for p in c.path],[p.y for p in c.path],'--',label=c.name)
ax.axis('equal');ax.legend();ax.set_title('Active course / red: runtime curvature > 1/1.15m')
ax.set_xlabel('East [m]');ax.set_ylabel('North [m]');bx.axhline(1/1.15,color='red',ls='--');bx.axhline(-1/1.15,color='red',ls='--');bx.set_xlabel('CSV row index');bx.set_ylabel('Runtime curvature [1/m]');bx.legend();fig.tight_layout();fig.savefig(Path(__file__).with_name('course.png'),dpi=160)
