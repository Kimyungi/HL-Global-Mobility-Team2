from pathlib import Path
import csv, re, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
font_manager.fontManager.addfont("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf")
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.backends.backend_pdf import PdfPages

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10,'figure.dpi':150})
SESSIONS=[('parking','주차','v2_20260917_230818_739351'),('obstacle','장애물 회피','v2_20260917_220254_439617')]
COLORS={'출발 대기':'#9298a3','안전 정지':'#c94747','ESTOP':'#9c173b','GPS 주행':'#2878b5','차선 주행':'#5f9fc5','주차 선택 대기':'#d6a11d','주차 전진':'#00a28a','후진 전환 대기':'#b28e29','주차 후진':'#8752b4','주차 완료 대기':'#de7a25','주차 출차':'#33954a','주차 종료':'#333333','주차 FAULT':'#dd2222','회피':'#e47724','GPS 복귀':'#9853a8','주차 제어':'#8553ad','주행 종료':'#222222'}
PHASE={'STOP_SELECT':'주차 선택 대기','ADVANCE_3':'주차 전진','STOP_REVERSE':'후진 전환 대기','REVERSE':'주차 후진','WAIT_10':'주차 완료 대기','EXIT':'주차 출차','EXIT_STOP':'주차 종료','DONE':'주차 종료','FAULT':'주차 FAULT'}
def read(p):
 with p.open() as f:return list(csv.DictReader(f))
def col(rows,key):return np.array([float(r[key]) for r in rows])
summary={}
with PdfPages(OUT/'parking_obstacle_report.pdf') as pdf:
 for key,title,session in SESSIONS:
  folder=ROOT/'drive_logs'/session
  d=read(OUT/(key+'_decoded.csv'));v=read(folder/'vehicle_vector.csv');t0=float(d[0]['time']);tm=col(d,'time');t=tm-t0
  vt=col(v,'stamp_s')-t0
  phase_events=[]
  if key=='parking':
   for line in (ROOT/'log_v2/ros/python3_50517_1789654099518.log').read_text().splitlines():
    m=re.search(r'\[([0-9]+\.[0-9]+)\].*T reference parking (\w+):',line)
    if m:phase_events.append((float(m[1]),m[2]))
  labels=[]
  for row in d:
   now=float(row['time'])
   phase=next((p for stamp,p in reversed(phase_events) if stamp<=now),None)
   if row['safety']=='4':label='ESTOP'
   elif row['top']=='0':label='출발 대기'
   elif row['safety']=='3':label='안전 정지'
   elif row['top']=='2':label='주행 종료'
   elif row['mission']=='1':label=PHASE.get(phase,'주차 제어')
   elif row['avoidance']=='1':label='회피'
   elif row['avoidance']=='3':label='GPS 복귀'
   else:label='차선 주행' if row['navigation']=='0' else 'GPS 주행'
   labels.append(label)
  labels=np.array(labels);bound=np.r_[0,np.flatnonzero(labels[1:]!=labels[:-1])+1,len(d)]
  xy=np.column_stack((col(d,'x'),col(d,'y')));valid=col(d,'position_valid')>0
  def timeplot(ax,which):
   if which=='steering':
    ax.plot(vt,np.rad2deg(col(v,'str_ref')),color='#d04f28',lw=1.4,label='목표 조향 (dSPACE str_ref)')
    ax.plot(vt,np.rad2deg(col(v,'str')),color='#226aaf',lw=1.1,label='실제 조향 (str)')
    ax.set_ylabel('조향각 [deg]');ax.set_title('시간에 따른 목표·실제 조향',loc='left',fontweight='bold')
   else:
    ax.plot(t,col(d,'v_ref'),color='#d04f28',lw=1.5,label='목표 속도 (MGM 스냅샷 복원)',drawstyle='steps-post')
    ax.plot(vt,col(v,'v'),color='#226aaf',lw=1.1,label='실제 속도 (dSPACE v)')
    ax.set_ylabel('속도 [m/s] · 음수=후진');ax.set_title('시간에 따른 목표·실제 속도',loc='left',fontweight='bold')
   for a,b in zip(bound[:-1],bound[1:]):ax.axvspan(t[a],t[min(b,len(t)-1)],color=COLORS[labels[a]],alpha=.07,lw=0)
   ax.axhline(0,color='#777777',lw=.6);ax.set_xlim(0,t[-1]);ax.set_xlabel('기록 시작 후 경과 시간 [s]');ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=9)
  def mapplot(ax):
   segments=np.stack([xy[:-1],xy[1:]],axis=1);ok=valid[:-1]&valid[1:]
   ax.add_collection(LineCollection(segments[ok],colors=[COLORS[s] for s in labels[:-1][ok]],linewidths=2.5))
   idx=np.flatnonzero(valid)[::10];ax.scatter(xy[idx,0],xy[idx,1],c=[COLORS[s] for s in labels[idx]],s=10,zorder=3)
   good=np.flatnonzero(valid)
   for i,name in [(good[0],'시작'),(good[-1],'기록 끝')]:
    ax.scatter(*xy[i],s=70,facecolors='white',edgecolors='black',zorder=4)
    ax.annotate(name,xy[i],xytext=(8,12),textcoords='offset points',fontsize=9,zorder=5)
   # Label mission phase changes, including changes made while stationary.
   changes=[]
   for a,b in zip(bound[:-1],bound[1:]):
    if valid[a] and (labels[a] in ('회피','GPS 복귀','ESTOP','주행 종료') or labels[a] in PHASE.values()):changes.append(a)
   for j,i in enumerate(changes):
    ax.annotate(f'{t[i]:.1f}s {labels[i]}',xy[i],xytext=(15 if j%2==0 else -110,24+18*(j%3)),textcoords='offset points',fontsize=7.5,arrowprops={'arrowstyle':'-','color':'#666','lw':.6},bbox={'fc':'white','alpha':.8,'ec':'none'},zorder=6)
   ax.autoscale();ax.margins(.22);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
   ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.set_title('GPS 차량 위치별 스테이트',loc='left',fontweight='bold')
   states=list(dict.fromkeys(labels));ax.legend(handles=[Line2D([0],[0],color=COLORS[s],lw=3,label=s) for s in states],loc='upper left',bbox_to_anchor=(1.01,1),fontsize=9)
  fig,axes=plt.subplots(3,1,figsize=(14,14),gridspec_kw={'height_ratios':[1,1,1.7]})
  fig.suptitle(f'{title} 주행 로그 | {session}',fontweight='bold',fontsize=17)
  timeplot(axes[0],'steering');timeplot(axes[1],'speed');mapplot(axes[2])
  fig.text(.05,.012,'조향/실속도: vehicle_vector.csv | 목표속도·상위 상태·GPS: mgm_snapshots.bin v38 | 주차 세부 상태: ROS 로그\n목표속도·상위 상태 복원 결과는 당시 transitions.csv와 대조 일치. 그래프 끝은 기록 끝이며 미션 완료를 뜻하지 않음.',fontsize=9,color='#444')
  fig.tight_layout(rect=(0,.045,1,.965));fig.savefig(OUT/(key+'_overview.png'));pdf.savefig(fig);plt.close(fig)
  for name in ['steering','speed','states']:
   fig,ax=plt.subplots(figsize=(12,6 if name=='states' else 4))
   if name=='states':mapplot(ax)
   else:timeplot(ax,name)
   fig.suptitle(title+' | '+session);fig.tight_layout();fig.savefig(OUT/(key+'_'+name+'.png'));plt.close(fig)
  with (OUT/(key+'_state_timeline.csv')).open('w') as f:
   w=csv.writer(f);w.writerow(['start_s','end_s','state','map_x','map_y'])
   for a,b in zip(bound[:-1],bound[1:]):w.writerow([t[a],t[min(b,len(t)-1)],labels[a],*xy[a]])
  summary[key]={'session':session,'duration_s':float(t[-1]),'snapshot_count':len(d),'vehicle_records':len(v),'states':list(dict.fromkeys(labels)),'target_speed_range':[float(col(d,'v_ref').min()),float(col(d,'v_ref').max())]}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
(OUT/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>주차 / 장애물 회피 주행 분석</title><style>body{font-family:sans-serif;max-width:1500px;margin:30px auto;background:#eee}img{width:100%}section{background:white;padding:20px;margin:20px 0}a{margin-right:20px}</style><h1>주차 · 장애물 회피 주행 플롯</h1><p>최근 각 주행 세션. 시간축은 기록 시작 기준. 음수 속도는 후진. 기록 종료는 미션 완료를 뜻하지 않습니다.</p><a href="parking_obstacle_report.pdf">PDF 내려받기</a>'''+''.join(f'<section><h2>{title} — {session}</h2><a href="{key}_steering.png">조향</a><a href="{key}_speed.png">속도</a><a href="{key}_states.png">위치별 스테이트</a><a href="{key}_state_timeline.csv">상태 시간표 CSV</a><img src="{key}_overview.png"></section>' for key,title,session in SESSIONS))
print(json.dumps(summary,ensure_ascii=False,indent=2))
