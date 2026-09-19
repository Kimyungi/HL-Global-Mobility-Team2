"""Synthetic production-callback audit. No ROS node, executor, or hardware opened.
Run after sourcing ROS + workspace; PYTHONPATH includes stack_avoid and its test dir.
Synthetic scenes are not reconstructed field scans. Timing is local wall-clock.
"""
import cProfile, json, math, pstats, time
from collections import Counter
from pathlib import Path
from test_surface_node import node as node_fixture, scene_scan

out=Path(__file__).resolve().parent
cases={
 'one_obstacle_2m':[((2.,-.2),(2.,.2))],
 'one_obstacle_3_3m':[((3.3,-.2),(3.3,.2))],
 'wall_2m':[((2.,-3.),(2.,3.))],
 'diagonal':[((.74,1.4),(3.24,.2))],
 'two_obstacles':[((2.,-.2),(2.,.2)),((3.2,.1),(3.2,.5))],
}
results=[]
for name,segments in cases.items():
 n=node_fixture.__wrapped__(); n.detect_range=3.5; n.detect_half_width=.5; n.offset_max=2.5
 scan=scene_scan(segments); counts=Counter(); original=n._planner._safe
 def check(points,obs):
  reason='empty_curve' if not points else 'curvature' if any(abs(p[3])>1/n._planner.min_radius+1e-6 for p in points) else None
  ok=original(points,obs); counts['safe' if ok else reason or 'footprint']+=1; return ok
 n._planner._safe=check
 goals=n._path_goals
 def count_goals(scan):
  t=time.perf_counter(); gs=goals(scan); counts['candidates']=len(gs); counts['goals_ms']=round((time.perf_counter()-t)*1000,3); return gs
 n._path_goals=count_goals
 pr=cProfile.Profile(); start=time.perf_counter(); pr.enable(); n.on_scan(scan); pr.disable(); elapsed=time.perf_counter()-start
 with (out/(name+'_profile.txt')).open('w') as f:pstats.Stats(pr,stream=f).strip_dirs().sort_stats('cumtime').print_stats(18)
 results.append(dict(scene=name,profiled_seconds=elapsed,reason=n._planner.reason,target=bool(n.messages[-1].points),counts=dict(counts)))
 # No profiling overhead in reported callback budget comparison.
 n2=node_fixture.__wrapped__();n2.detect_range=3.5;n2.detect_half_width=.5;n2.offset_max=2.5
 start=time.perf_counter();n2.on_scan(scan);results[-1]['unprofiled_seconds']=time.perf_counter()-start
 print(json.dumps(results[-1]),flush=True)
(out/'synthetic_audit.json').write_text(json.dumps(results,indent=2))
