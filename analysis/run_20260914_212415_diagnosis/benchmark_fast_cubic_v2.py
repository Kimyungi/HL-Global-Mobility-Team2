import random,math,time,json,statistics,struct
from pathlib import Path
import stack_avoid.gps_cubic_path as mod
from fast_cubic_v2 import cubic_connector as fast,array_basis
from fast_cubic_prototype import basis
from test_surface_node import node as fixture,scene_scan
old=mod.cubic_connector
rng=random.Random(2141);pairs=[]
for i in range(250):
 a=(rng.uniform(-3,3),rng.uniform(-3,3),rng.uniform(-math.pi,math.pi),0.)
 b=(rng.uniform(-3,8),rng.uniform(-3,3),rng.uniform(-math.pi,math.pi),0.)
 pairs.append((a,b,.05,(1.,1.25,1.5,1.75)[i%4]))
pairs.extend([((1.,0.,0.,0.),(2.66,s*.76,0.,0.),.05,k) for s in [-1,1] for k in [1.,1.25,1.5,1.75]])
points=0; bit_equal=True
for args in pairs:
 a,b=old(*args),fast(*args)
 assert len(a)==len(b)
 for x,y in zip(a,b):
  points+=1
  assert struct.pack('dddd',*x)==struct.pack('dddd',*y), (x,y)
result={'connector_cases':len(pairs),'points_bit_identical':points,'scenes':[]}
for gap in [3.3,2.,.7]:
 scan=scene_scan([((gap,-.2),(gap,.2))]); outputs={}
 for name,func in [('baseline',old),('prototype',fast)]:
  mod.cubic_connector=func; times=[];cold=[]
  for i in range(7):
   n=fixture.__wrapped__();n.detect_range=3.5;n.detect_half_width=.5;n.offset_max=2.5
   if name=='prototype' and i==0:
    basis.cache_clear();array_basis.cache_clear()
   start=time.perf_counter();n.on_scan(scan);dt=time.perf_counter()-start
   if i==0:cold.append(dt)
   else:times.append(dt)
  outputs[name]=(n._planner.path.points if n._planner.path else None,n._planner.control_target,n._planner.reason)
  result['scenes'].append(dict(gap_m=gap,method=name,first_ms=cold[0]*1000,median_ms=statistics.median(times)*1000))
 assert outputs['baseline']==outputs['prototype']
mod.cubic_connector=old
print(json.dumps(result,indent=2));Path(__file__).with_name('fast_cubic_v2_results.json').write_text(json.dumps(result,indent=2))
