"""Historical v28 values, matched at each recorded MGM timestamp; no new replay."""
import csv, math
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
def read(p):return list(csv.DictReader(p.open()))
a=read(OUT/'traffic_inputs.csv')
old=read(ROOT/'analysis/run_20260914_063951_diagnosis/inputs.csv')
b=read(ROOT/'analysis/run_20260914_063951_diagnosis/replay.csv')
tr=read(ROOT/'drive_logs/v2_20260914_063951_118844/transitions.csv')
assert len(a)==len(b)==len(old)==18310
for x,y in zip(a,old):
 assert x['tick']==y['tick'] and x['event_time_ns']==y['event_time_ns']
 assert x['vehicle_speed']==y['vehicle_speed'] and x['vehicle_speed_valid']==y['vehicle_speed_valid']
for row in tr:
 r=b[int(row['tick'])]
 for k,v in row.items():
  key={'reference_source':'path_source'}.get(k,k)
  assert math.isclose(float(v),float(r[key]),rel_tol=1e-5,abs_tol=1e-5),(row['tick'],k)
end=next(i for i in range(3400,len(b)) if b[i]['signal']=='3' and b[i-1]['signal']!='3')
start=next(i for i in range(3300,end) if a[i]['stopline_detected']=='1' and a[i-1]['stopline_detected']=='0')
seed=next(i for i in range(start,end) if a[i]['stopline_detected']=='0' and a[i-1]['stopline_detected']=='1')
zero=next(i for i in range(start,end) if float(b[i]['v_ref'])==0)
t0=int(a[start]['event_time_ns'])/1e9
kst=lambda i:datetime.fromtimestamp(int(a[i]['event_time_ns'])/1e9,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
rows=[]
for i in range(end+1):
 t=int(a[i]['event_time_ns'])/1e9
 if t<t0-2:continue
 rows.append(dict(tick=i,event_time_ns=a[i]['event_time_ns'],time_kst=kst(i),seconds_from_detection=t-t0,
  v_ref=float(b[i]['v_ref']),v_act=float(a[i]['vehicle_speed']) if a[i]['vehicle_speed_valid']=='1' else '',
  stopline_detected=int(a[i]['stopline_detected']),remaining_m=float(b[i]['traffic_remaining_m']) if i>=seed else '',
  distance_known=int(i>=seed),signal=int(b[i]['signal'])))
with (OUT/'aligned.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
font=next((f.name for f in font_manager.fontManager.ttflist if f.name=='Noto Sans CJK JP'),None)
if font:plt.rcParams['font.family']=font
plt.rcParams['axes.unicode_minus']=False
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,(ax1,ax2)=plt.subplots(2,1,figsize=(14,8.4),sharex=True,layout='constrained',gridspec_kw={'height_ratios':[1.15,1]})
x=[r['seconds_from_detection'] for r in rows]
ax1.step(x,[r['v_ref'] for r in rows],where='post',label='v_ref · 차량 목표 속도 (당시 코어 재생값)',color='#1976d2',lw=2.6)
ax1.plot(x,[r['v_act'] if r['v_act']!='' else math.nan for r in rows],label='v_act · 실측 속도 (유효 입력)',color='#ee812c',lw=2.3)
ax1.set_ylabel('속도 [m/s]');ax1.set_ylim(-.12,2.8);ax1.set_yticks([0,.5,1,1.5,2]);ax1.legend(loc='upper left',fontsize=10)
ax2.plot(x,[r['remaining_m'] if r['remaining_m']!='' else math.nan for r in rows],color='#15876c',lw=2.6,label='정지선 잔여거리 · 제어기 추정값')
ax2.axhline(0,color='#34495e',lw=1,ls='--');ax2.set_ylabel('남은 거리 [m]');ax2.set_ylim(-1.45,2.05)
seed_t=int(a[seed]['event_time_ns'])/1e9-t0
ax2.axvspan(-2,seed_t,color='#eef1f5')
ax2.text(-.95,.45,'거리 미확정\n(0m가 아님)',ha='center',va='center',color='#607080',fontsize=13)
ax2.legend(loc='upper left',fontsize=10)
events=[(start,'인식', '#4266a1'),(seed,'거리 계산 시작','#b77c2b'),(zero,'v_ref = 0','#8763ab'),(end,'정차','#ca4b4b')]
for i,label,color in events:
 tx=int(a[i]['event_time_ns'])/1e9-t0
 for ax in [ax1,ax2]:ax.axvline(tx,color=color,ls=':',lw=1.4,alpha=.85)
 if i in [start,zero,end]:
  ax1.annotate(label+'\n'+kst(i),xy=(tx,2.06 if i==start else .02),xytext=(tx+(-.12 if i==end else -.1),2.2 if i==start else .75),ha='right' if i==end else 'center',fontsize=10,color=color,arrowprops={'arrowstyle':'-','color':color})
for i,label in [(seed,'1.500 m\n'+kst(seed)),(zero,'−0.607 m'),(end,'−1.042 m')]:
 tx=int(a[i]['event_time_ns'])/1e9-t0;v=float(b[i]['traffic_remaining_m'])
 ax2.scatter(tx,v,c='#15876c',s=32,zorder=5)
 ax2.annotate(label,(tx,v),xytext=(8,10) if i!=end else (-12,-23),textcoords='offset points',ha='right' if i==end else 'left',fontsize=11,fontweight='bold')
for ax in [ax1,ax2]:ax.grid(alpha=.2);ax.set_xlim(-2,x[-1]+.16)
ax2.set_xlabel('정지선 인식 기준 경과 시간 [s] · 0초 = 06:40:25.826 KST')
ax2.set_xticks([-2,-1.5,-1,-.5,0,.5,1,1.5,2,2.5,round(x[-1],2)])
fig.suptitle('정지선 인식 2초 전부터 정차까지 · 속도와 잔여거리\n2026-09-14 06:40:23.826–06:40:28.656 KST | 동일 MGM 타임스탬프 (약 10ms)',fontsize=17)
fig.supxlabel('잔여거리는 실측이 아닌 제어기 추정: 정지선 검출 소실 시 1.5m로 설정 후 속도 적분으로 차감. 음수 = 추정 정지선 통과.\n정차 기준: STOPPED_WAIT 진입 · 당시 v_act = 0.000565 m/s. v_ref는 원본 상태 전이 42행과 대조한 당시 v28 재생값.',fontsize=10,color='#536477')
fig.savefig(OUT/'traffic_stop_timeline.png',dpi=180)
fig.savefig(OUT/'traffic_stop_timeline.svg')
for i,label,_ in events:print(label,kst(i),'v_ref',b[i]['v_ref'],'v_act',a[i]['vehicle_speed'],'remaining',b[i]['traffic_remaining_m'])
print('Aligned samples',len(rows),'max dt',max(y['seconds_from_detection']-x['seconds_from_detection'] for x,y in zip(rows,rows[1:])))
