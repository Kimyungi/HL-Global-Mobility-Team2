from pathlib import Path
import csv,json,html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import font_manager
O=Path(__file__).resolve().parent; R=O.parents[1]; S=R/'drive_logs/v2_20260918_211819_564358'
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10,'figure.dpi':140})
def read(p):
 with open(p) as f:return list(csv.DictReader(f))
inputs=read(O/'inputs_all.csv'); replay=read(O/'replay_all.csv'); trans=read(S/'transitions.csv')
# Compare reconstructed outputs against every recorded transition before plotting.
checks=0
for row in trans:
 r=replay[int(row['tick'])]
 for k,v in row.items():
  key={'reference_source':'path_source'}.get(k,k)
  if key in r:
   assert np.isclose(float(v),float(r[key]),atol=1e-4,rtol=1e-4),(row['tick'],k,v,r[key])
   checks+=1
cut=next((i for i,r in enumerate(replay) if int(r['route_index'])>0),len(replay))
d=[dict(a,**{k:v for k,v in b.items() if k not in a}) for a,b in zip(inputs[:cut],replay[:cut])]
def c(k):return np.array([float(r[k]) for r in d])
t0=c('event_time_ns')[0]/1e9;t=c('event_time_ns')/1e9-t0;end=t0+t[-1]
for row,tm in zip(d,t):row['elapsed_s']=tm
with open(O/'route04_all_metrics.csv','w') as f:
 w=csv.DictWriter(f,fieldnames=list(d[0]));w.writeheader();w.writerows(d)
v=[r for r in read(S/'vehicle_vector.csv') if t0<=float(r['stamp_s'])<=end]
l=[r for r in read(S/'lateral.csv') if t0<=float(r['stamp_s'])<=end and r['route_id']=='04']
def arr(rows,k):return np.array([float(r[k]) for r in rows])
labels=[]
for r in d:
 if r['top']=='0':s='출발 대기'
 elif r['safety']=='4':s='ESTOP'
 elif r['safety']=='3':s='안전 정지'
 elif r['reference_motion_blocked']=='1':s='회피 ref 정지' if r['avoidance']=='1' else 'ref 정지'
 elif r['avoidance']=='1':s='회피 주행'
 elif r['avoidance']=='3':s='GPS 복귀'
 else:s='차선 주행' if r['navigation']=='0' else 'GPS 주행'
 labels.append(s)
labels=np.array(labels);states=list(dict.fromkeys(labels));colors=dict(zip(states,plt.get_cmap('tab10').colors))
bounds=np.r_[0,np.where(labels[1:]!=labels[:-1])[0]+1,len(d)]
events=[]
for a,b in zip(bounds[:-1],bounds[1:]):events.append({'start_s':t[a],'end_s':t[b] if b<len(t) else t[-1],'state':labels[a],'x':c('gps_x')[a],'y':c('gps_y')[a],'idx':int(c('gps_track_index')[a])})
with open(O/'state_intervals.csv','w') as f:
 w=csv.DictWriter(f,fieldnames=list(events[0]));w.writeheader();w.writerows(events)
figures=[]
pdf=PdfPages(O/'route04_report.pdf')
def save(fig,name,title):
 fig.suptitle(title+' | 마지막 주행 · 경로 04',fontsize=15)
 fig.tight_layout(rect=(0,.015,1,.96));fig.savefig(O/(name+'.png'));pdf.savefig(fig);plt.close(fig);figures.append((name,title))
def timeline(groups,name,title):
 fig,axes=plt.subplots(len(groups),1,figsize=(14,3*len(groups)),sharex=True,squeeze=False)
 for ax,(keys,ylabel) in zip(axes[:,0],groups):
  for k,label in keys:ax.plot(t,c(k),label=label,lw=1)
  ax.set_ylabel(ylabel);ax.grid(alpha=.25);ax.legend(loc='upper right',fontsize=8);ax.set_xlim(t[0],t[-1])
 axes[-1,0].set_xlabel('세션 시작 후 시간 [s]');save(fig,name,title)
valid=c('gps_position_valid')>0
fig,axs=plt.subplots(1,2,figsize=(16,8))
for state in states:
 m=valid&(labels==state);axs[0].scatter(c('gps_x')[m],c('gps_y')[m],s=8,color=colors[state],label=state)
event_number=0
for j,e in enumerate(events):
 if j==0 or e['end_s']-e['start_s']>1:
  event_number+=1
  axs[0].annotate(f"{event_number}. {e['start_s']:.1f}s {e['state']} (idx {e['idx']})",(e['x'],e['y']),xytext=(.02,.67-event_number*.045),textcoords='axes fraction',fontsize=7,arrowprops={'arrowstyle':'-','lw':.5,'color':'#888'},bbox={'fc':'white','alpha':.9,'ec':'none'})

p=axs[1].scatter(c('gps_x')[valid],c('gps_y')[valid],c=c('vehicle_speed')[valid],s=10,cmap='viridis');fig.colorbar(p,ax=axs[1],label='실제 속도 [m/s]')
for ax in axs:
 ax.set_aspect('equal');ax.set_xlabel('기록된 map X [m]');ax.set_ylabel('기록된 map Y [m]');ax.grid(alpha=.25)
 for i,txt in [(np.flatnonzero(valid)[0],'기록 시작'),(np.flatnonzero(valid)[-1],'04 기록 끝')]:ax.plot(c('gps_x')[i],c('gps_y')[i],'kx');ax.annotate(txt,(c('gps_x')[i],c('gps_y')[i]),fontsize=8)
axs[0].legend(fontsize=8);axs[0].set_title('위치별 제어 상태 / 정차');axs[1].set_title('위치별 실제 속도')
save(fig,'01_position','위치 · 상태 · 속도 지도')
fig,axs=plt.subplots(3,1,figsize=(14,10),sharex=True)
axs[0].plot(t,c('v_ref'),label='MGM 목표속도');axs[0].plot(arr(v,'stamp_s')-t0,arr(v,'v'),label='CAN 실제속도',alpha=.8);axs[0].set_ylabel('m/s')
axs[1].plot(arr(v,'stamp_s')-t0,np.rad2deg(arr(v,'str_ref')),label='dSPACE 목표조향');axs[1].plot(arr(v,'stamp_s')-t0,np.rad2deg(arr(v,'str')),label='실제조향',alpha=.8);axs[1].set_ylabel('deg')
for j,s in enumerate(states):
 m=labels==s;axs[2].scatter(t[m][::5],np.full(m.sum(),j)[::5],s=3,color=colors[s])
axs[2].set_yticks(range(len(states)),states);axs[2].set_xlabel('세션 시작 후 시간 [s]')
for ax in axs:ax.grid(alpha=.25);ax.set_xlim(0,t[-1])
for ax in axs[:2]:ax.legend()
save(fig,'02_motion','목표·실제 속도 / 조향 / 상태 시간표')
timeline([([('gps_track_index','현재 idx')],'idx'),([('gps_cross_track','횡오차')],'m'),([('gps_station_yaw_error','경로 접선 대비 헤딩오차')],'rad'),([('gps_fix_quality','GNSS fix quality'),('gps_heading_valid','heading valid')],'상태')],'03_tracking','경로 진행과 위치 추종 오차')
timeline([([('avoid_obstacle_detected','장애물'),('avoid_avoidable','회피 가능'),('avoid_narrow_gap','좁은 통로')],'판정'),([('avoid_points','회피 ref 개수'),('avoid_maneuver_done','회피 완료')],'개수 / 판정'),([('avoid_v_suggest','회피 요청속도'),('v_ref','최종 목표속도')],'m/s'),([('avoid_age_s','회피 ref age'),('avoid_timeout_s','허용 age')],'s')],'04_avoidance','회피 인지와 경로 생성 · 정지 원인')
timeline([([('ref_valid','선택 ref 유효'),('ref_fresh','선택 ref 신선'),('reference_motion_blocked','ref 주행 차단'),('immediate_stop','즉시 정지')],'판정'),([('ref_age_s','선택 ref age')],'s'),([('autonomous_enabled','출발 인가'),('external_stop','외부 정지'),('stop_reasons','안전 정지 비트마스크')],'값'),([('path_source','ref 소스'),('speed_owner','속도 결정 주체')],'enum')],'05_reference','ref 유효성과 정지 게이트')
timeline([([('top','상위'),('navigation','항법'),('avoidance','회피'),('safety','안전'),('mission','미션'),('signal','신호등')],'enum'),([('lane_confidence','차선 신뢰도'),('camera_available','카메라'),('gps_fixed_ready','GPS 준비'),('lidar_valid','LiDAR')],'값'),([('sensor_alive_mask','센서 비트마스크'),('gps_position_valid','위치 유효'),('vehicle_speed_valid','차속 유효')],'값')],'06_states_sensors','전체 상위 상태 및 센서 유효성')
timeline([([('front_clearance_m','전방'),('left_clearance_m','좌측'),('right_clearance_m','우측')],'차체 외곽 거리 [m]'),([('estop_active','ESTOP'),('traffic_red_active','빨강'),('traffic_green_active','초록'),('traffic_stopline_detected','정지선')],'판정'),([('parking_v_suggest','주차 요청속도'),('parking_done','주차 완료'),('parking_path_blocked','주차 경로 차단')],'값')],'07_safety_missions','장애물 거리 / ESTOP / 신호등 / 주차 입력')
# All recorded and reconstructed scalar metrics, including constant channels, in an appendix.
keys=[k for k in d[0] if k not in ('tick','event_time_ns','elapsed_s') and not any(x in k for x in ['generation','request_id','instance_id','sequence_id'])]
for offset in range(0,len(keys),12):
 fig,axs=plt.subplots(6,2,figsize=(16,15),sharex=True)
 for ax,k in zip(axs.flat,keys[offset:offset+12]):
  vals=c(k); vals=np.where(np.isfinite(vals),vals,np.nan);ax.plot(t,vals,lw=.7);ax.set_title(k,fontsize=9);ax.grid(alpha=.2)
 for ax in list(axs.flat)[len(keys[offset:offset+12]):]:ax.axis('off')
 save(fig,f'appendix_{offset//12+1:02}','전체 수치 채널 · 단위/enum은 원본 필드 기준')
# Additional file logs, cropped to the same route04 wall-time window.
for filename,clock,scale in [('lateral.csv','stamp_s',1),('zone_observations.csv','time_ns',1e-9),('mgm_jitter.csv','# window_end_epoch_us',1e-6)]:
 rows=[r for r in read(S/filename) if t0<=float(r[clock])*scale<=end]
 if not rows:continue
 with open(O/filename,'w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 numeric=[]
 for k in rows[0]:
  if k==clock or any(n in k for n in ['sequence_id','generation','time_ns']):continue
  try:arr(rows,k)
  except ValueError:continue
  numeric.append(k)
 for offset in range(0,len(numeric),8):
  fig,axs=plt.subplots(4,2,figsize=(16,12),sharex=True)
  for ax,k in zip(axs.flat,numeric[offset:offset+8]):
   ax.plot(arr(rows,clock)*scale-t0,arr(rows,k),lw=.8);ax.set_title(k);ax.grid(alpha=.2)
  for ax in list(axs.flat)[len(numeric[offset:offset+8]):]:ax.axis('off')
  save(fig,filename[:-4]+f'_{offset//8+1:02}',filename+' 원본 기록 (jitter 단위 us)')

pdf.close()
summary={'session':S.name,'route':'04','end_rule':'first route_index > 0, before route 05','duration_s':float(t[-1]),'ticks':len(d),'idx_range':[int(c('gps_track_index')[valid].min()),int(c('gps_track_index').max())],'transition_value_checks':checks,'transition_mismatches':0,'states':states,'target_zero_duration_s':float(np.sum(np.diff(t,append=t[-1])[np.abs(c('v_ref'))<.001])),'notes':['목표속도 및 상태는 기록 당시 파라미터와 로컬 빌드 core_replay로 복원하고 transitions.csv와 대조함.','05 구간 ESTOP은 범위 밖이므로 제외.','map 좌표는 해당 세션 GPS 원점 기준. CSV east/north와 원점을 동일하다고 가정하지 않음.','LiDAR +inf는 유한 장애물 거리 없음. 미수신/무효 입력은 별도 validity로 확인.','주차 세부 phase 및 원시 영상/점군은 이 스냅샷에 없어 복원하지 않음. lane_frames.csv와 mission_events.csv는 헤더만 있어 그래프 없음.','입력 스냅샷 수신값에는 이전 메시지의 유지값이 포함될 수 있음. age/valid와 함께 해석.']}
(O/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
text='''# 마지막 주행 · 경로 04 분석\n\n세션: '''+S.name+f'''\n\n범위: {t[-1]:.2f}초, {len(d)}틱. 05 진입 직전까지.\n\n검증: transitions.csv의 {checks}개 값과 복원 출력 대조 일치.\n\n주요 파일: index.html, route04_report.pdf, route04_all_metrics.csv, state_intervals.csv.\n\nref source: 0=LINE, 1=GPS, 2=AVOID, 3=PARKING, 4=RECOVERY.\n속도 주체: 0=NAVIGATION, 1=AVOIDANCE, 2=TRAFFIC, 3=MISSION, 4=SAFETY, 5=FINISH.\nNav: 0=LINE, 1=GPS_BACKUP, 2=GPS_ONLY. Avoid: 0=INACTIVE, 1=ACTIVE, 3=GPS_RETURN. Safety: 0=NORMAL, 3=SAFE_STOP, 4=ESTOP.\n\n'''+ '\n'.join('- '+n for n in summary['notes'])
(O/'README.md').write_text(text)
(O/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>마지막 주행 04 분석</title><style>body{font-family:sans-serif;max-width:1500px;margin:30px auto;background:#eee}section{background:white;padding:20px;margin:20px 0}img{width:100%}a{margin-right:20px}</style><h1>마지막 주행 · 경로 04</h1><pre>'+html.escape(text)+'</pre><a href="route04_report.pdf">전체 PDF</a><a href="route04_all_metrics.csv">전체 수치 CSV</a><a href="state_intervals.csv">상태 전이 CSV</a>'+''.join(f'<section><h2>{title}</h2><a href="{name}.png">원본 PNG</a><img loading="lazy" src="{name}.png"></section>' for name,title in figures))
print(json.dumps(summary,ensure_ascii=False,indent=2))
