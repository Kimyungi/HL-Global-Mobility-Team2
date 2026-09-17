import sys,json,subprocess,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'src/stack_gps'))
from stack_gps.path_engine import PathEngine,load_waypoints_csv
out=[]
for hz in (5,10):
 for phase in (0.,.25,.5,.75):
  e=PathEngine(load_waypoints_csv(root/'src/stack_gps/waypoints/waypoints_halla_20260916_path_03.csv'),station_tracking=True)
  hits=[]
  for n,t in enumerate(np.arange(phase/hz,e.station[-1]/2,1/hz)):
   station=2*t;x=np.interp(station,e.station,e.e);y=np.interp(station,e.station,e.n)
   lat=e._lat0+y/111320;lon=e._lon0+x/e._m_per_deg_lon
   yaw=np.interp(station,e.station,np.unwrap(e.yaw))
   snap=e.snapshot(lat,lon,heading=float(yaw),now=float(t),v_ref=2.,sample_time=1/hz,generation=n+1)
   hits.append(int(145<=snap['idx']<=149))
  r=subprocess.run(['/tmp/v0916_zone_probe'],input='\n'.join(map(str,hits)),text=True,capture_output=True,check=True)
  count,entered=map(int,r.stdout.split());out.append(dict(hz=hz,phase=phase,samples_in_zone=count,zone_entry_events=entered))
Path(__file__).with_name('zone_sampling.json').write_text(json.dumps(out,indent=2));print(out)
