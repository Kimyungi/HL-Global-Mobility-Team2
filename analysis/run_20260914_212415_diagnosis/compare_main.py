"""Same synthetic callbacks; origin/main source loaded read-only with git show."""
import importlib.util,subprocess,time,json,types,math
from pathlib import Path
from test_surface_node import node as fixture,scene_scan
from stack_avoid.gps_cubic_path import WaypointWindow
base=Path(__file__).resolve().parent
source=subprocess.check_output(['git','show','origin/main:src/stack_avoid/stack_avoid/node.py'],text=True)
m=types.ModuleType('historical_main_node');exec(compile(source,'origin/main/node.py','exec'),m.__dict__)
cases=[]
for gap,cross,yaw in [(3.3,0.,0.),(2.,0.,0.),(.7,0.,0.),(3.3,.8,.3),(2.,.8,.3),(.7,.8,.3),(2.,-.8,-.3)]:
 scan=scene_scan([((gap,-.2),(gap,.2))])
 for branch in ['main','v2']:
  n=fixture.__wrapped__();n.detect_range=3.5;n.detect_half_width=.5;n.offset_max=2.5
  n._poses={'gps':((0.,cross,yaw),99_900_000_000)}
  n._gps_waypoints=WaypointWindow(list(map(float,range(13))),[(float(i),0.,0.,0.) for i in range(13)],0.)
  if branch=='main':
   for name in ['on_scan','_gap_target','_rate_limit','_nearest_front_obstacle','_behind_surface','_front_only_scan']:
    setattr(n,name,types.MethodType(getattr(m.StackAvoidNode,name),n))
  start=time.perf_counter();n.on_scan(scan);elapsed=time.perf_counter()-start
  p=n.messages[-1].points
  cases.append(dict(branch=branch,gap=gap,cross=cross,yaw=yaw,seconds=elapsed,target=[p[0].x,p[0].y,p[0].yaw,p[0].curvature] if p else None,reason=n._planner.reason if branch=='v2' else 'gap target'))
  print(json.dumps(cases[-1]),flush=True)
(base/'main_comparison.json').write_text(json.dumps(cases,indent=2))
