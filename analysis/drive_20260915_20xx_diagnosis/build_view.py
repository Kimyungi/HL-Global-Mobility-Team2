"""Read-only offline view: recorded positions + ABI-matched MGM replay, verified against transitions."""
import csv,json,bisect,math,collections
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];KST=ZoneInfo('Asia/Seoul')
C={'WAIT':'#94a3b8','GPS':'#2563eb','AVOID':'#16a34a','AVOID_HOLD':'#ea580c','ESTOP':'#dc2626','MANUAL':'#a855f7','LANE':'#06b6d4'}
def read(p):return list(csv.DictReader(p.open()))
def clock(t):return datetime.fromtimestamp(t,KST).strftime('%H:%M:%S.%f')[:-3] if t else None
def state(r,manual):
 if manual:return 'MANUAL'
 if r['top']!=1:return 'WAIT'
 if r['safety']!=0:return 'ESTOP'
 if r['source']==2:return 'AVOID_HOLD' if r['blocked'] else 'AVOID'
 return 'GPS' if r['source']==1 else 'LANE'
def trp(x,y,yaw,p):return [x+math.cos(yaw)*p[0]-math.sin(yaw)*p[1],y+math.sin(yaw)*p[0]+math.cos(yaw)*p[1]]
result=[]
for p in sorted(OUT.glob('*_decoded.csv')):
 name=p.name.removesuffix('_decoded.csv');run=ROOT/'drive_logs'/name
 rows=[{k:float(v) for k,v in r.items()} for r in read(p)];times=[r['t'] for r in rows]
 transitions=read(run/'transitions.csv');mismatches=[]
 for tr in transitions:
  r=rows[int(tr['tick'])]
  for a,b in [('top','top'),('navigation','nav'),('avoidance','avoid'),('safety','safety'),('reference_source','source')]:
   if int(tr[a])!=int(r[b]):mismatches.append([tr['tick'],a,tr[a],r[b]])
 assert not mismatches,mismatches
 first=next(r['t'] for r in rows if r['top']==1)
 rev=None;start=None
 for r in rows:
  if r['t']<first:continue
  if r['v']<-.2 and r['v_valid']:
   if start is None:start=r['t']
   if r['t']-start>=.3:rev=start;break
  else:start=None
 gps=read(run/'lateral.csv');gt=[float(g['stamp_s']) for g in gps]
 active=[r for r in rows if r['top']==1 and (rev is None or r['t']<rev)]
 end=min(times[-1], (rev+15 if rev else active[-1]['t']+10))
 frames=[]
 for g in gps:
  t=float(g['stamp_s'])
  if not first-2<=t<=end:continue
  i=bisect.bisect_right(times,t)-1
  if i<0:continue
  r=rows[i]
  if not r['gps_position_valid']:continue
  yaw=math.radians(float(g['heading_deg']));manual=rev is not None and t>=rev
  f={k:r[k] for k in ['tick','top','nav','avoid','safety','source','v','v_ref','ref_age','blocked','obstacle','avoid_n','avoid_age','avoid_timeout','gps_point_x','gps_point_y','gps_yaw_error','gps_heading_valid','target_n']}
  f.update(t=t,time=clock(t),xy=[r['gps_x'],r['gps_y']],yaw=yaw,route=g['route_id'],idx=int(g['idx']),station=float(g['station_m']),cross=float(g['cross_track_m']),quality=int(g['quality']),heading=g['heading_src'],fix_age=float(g['fix_age_s']),manual=manual,label=state(r,manual),match_age_ms=round((t-r['t'])*1000,2))
  f['target']=trp(*f['xy'],yaw,[r['target_point_x'],r['target_point_y']]) if r['target_n'] and not r['blocked'] else None
  f['gps_target']=trp(*f['xy'],yaw,[r['gps_point_x'],r['gps_point_y']]) if r['gps_n'] else None
  frames.append(f)
 events=[];prev=None
 for r in rows:
  if not first-2<=r['t']<=end:continue
  key=tuple(r[k] for k in ['top','source','avoid','safety','route_index'])
  if key!=prev:
   label=state(r,rev is not None and r['t']>=rev)
   events.append(dict(t=r['t'],time=clock(r['t']),label=label,route=['01','03','04','05','07'][int(r['route_index'])],text=f"{label} · route {int(r['route_index'])+1}"));prev=key
 if rev:events.append(dict(t=rev,time=clock(rev),label='MANUAL',text='후진 시작 → 이후 수동 조작'))
 events.sort(key=lambda e:e['t'])
 # GPS receiver statistics only in actual autonomous DRIVE, before manual cut.
 driving_g=[]
 for g in gps:
  t=float(g['stamp_s']);i=bisect.bisect_right(times,t)-1
  if i>=0 and rows[i]['top']==1 and (rev is None or t<rev):driving_g.append(g)
 phases=collections.Counter(state(r,False) for r in active)
 # Separate actual in-route lateral offset from distance beyond a route endpoint.
 routes={}
 for rid in ['01','03']:
  routes[rid]=[[float(x['east_m']),float(x['north_m'])] for x in read(ROOT/f'src/stack_gps/waypoints/waypoints_halla_reference_path_{rid}.csv')]
 summary=dict(name=name,first_go=clock(first),manual_start=clock(rev),manual_t=rev,drive_s=round(len(active)*.01,2),state_seconds={k:round(v*.01,2) for k,v in phases.items()},fix_counts=dict(collections.Counter(g['quality'] for g in driving_g)),heading_counts=dict(collections.Counter(g['heading_src'] for g in driving_g)),max_cross=round(max(r['cross'] for r in active),3),transition_check_count=len(transitions),blocked_seconds=round(sum(bool(r['blocked']) for r in active)*.01,2),route_points_behind_seconds=round(sum(r['source']==1 and r['gps_point_x']<0 and r['v_ref']>0 for r in active)*.01,2))
 result.append(dict(summary=summary,frames=frames,events=events,routes=routes))
 print(json.dumps(summary,ensure_ascii=False))
(OUT/'data.json').write_text(json.dumps(result,ensure_ascii=False,allow_nan=False))
(OUT/'summary.json').write_text(json.dumps([x['summary'] for x in result],ensure_ascii=False,indent=2))
html=(OUT/'viewer_template.html').read_text();(OUT/'index.html').write_text(html.replace('__DATA__',json.dumps(result,ensure_ascii=False,allow_nan=False)))
# Shareable static artifact, same frame data and classifications as the viewer.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.font_manager import FontProperties,findfont
font=FontProperties(family='Noto Sans CJK JP');plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(2,3,figsize=(17,10),gridspec_kw={'height_ratios':[3,1]})
for j,run in enumerate(result):
 ax=axs[0,j];fs=run['frames'];xs=[f['xy'][0] for f in fs];ys=[f['xy'][1] for f in fs]
 for rid,pts in run['routes'].items():
  ax.plot([p[0] for p in pts],[p[1] for p in pts],color='#cbd5e1',ls='--',lw=2,label='GPS CSV '+rid)
 for label in C:
  seg=[[a['xy'],b['xy']] for a,b in zip(fs,fs[1:]) if b['label']==label]
  if seg:ax.add_collection(LineCollection(seg,colors=C[label],linewidths=3,linestyles='dashed' if label=='MANUAL' else 'solid',label=label))
 ax.scatter(xs[0],ys[0],c='#0f172a',s=35,zorder=9);ax.annotate('GO',fs[0]['xy'],xytext=(5,8),textcoords='offset points')
 cut=next((f for f in fs if f['manual']),None)
 if cut:ax.scatter(*cut['xy'],c=C['MANUAL'],marker='X',s=100,zorder=10);ax.annotate('수동 시작\n'+cut['time'],cut['xy'],xytext=(10,10),textcoords='offset points',fontsize=9)
 margin=1.5;ax.set_xlim(min(xs)-margin,max(xs)+margin);ax.set_ylim(min(ys)-margin,max(ys)+margin);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('East (m)');ax.set_ylabel('North (m)')
 ax.set_title(run['summary']['name'][12:18]+' 세션\nGO '+run['summary']['first_go'],fontsize=12);ax.legend(fontsize=8,loc='best')
 t0=fs[0]['t'];ts=[f['t']-t0 for f in fs];b=axs[1,j];b.plot(ts,[f['v_ref'] for f in fs],c='#0f172a',label='MGM 명령속도');b.plot(ts,[f['v'] for f in fs],c='#2563eb',label='CAN 실측속도')
 if cut:b.axvspan(cut['t']-t0,ts[-1],color=C['MANUAL'],alpha=.12)
 b.axhline(0,color='#94a3b8',lw=.6);b.set_ylabel('m/s');b.set_xlabel('GO 2초 전부터 경과 (s)');b.grid(alpha=.2);b.legend(fontsize=8)
fig.suptitle('2026-09-15 20시대 · 차량 GPS 위치와 MGM 상태\n보라색 점선은 후진 시작 이후 수동 조작 — 자율주행 원인 분석에서 제외',fontsize=16)
fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/'overview.png',dpi=160);plt.close(fig)
