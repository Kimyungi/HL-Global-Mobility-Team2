"""Saved CSV: common lat/lon origin, separate route and zone views."""
from pathlib import Path
import csv, hashlib, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
font_manager.fontManager.addfont("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf")
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SOURCE = ROOT/'src/stack_gps/waypoints/yongin_no_parking.csv'
data = SOURCE.read_bytes()
(OUT/'source.csv').write_bytes(data)
rows = list(csv.DictReader(data.decode('utf-8-sig').splitlines()))
lat0, lon0 = float(rows[0]['lat']), float(rows[0]['lon'])
sx = 111320*np.cos(np.deg2rad(lat0))
paths = {}
for pid in range(1,8):
    rr = [r for r in rows if int(r['path_id']) == pid]
    xy = np.array([[(float(r['lon'])-lon0)*sx, (float(r['lat'])-lat0)*111320] for r in rr])
    zones = np.array([int(r['zone_id']) if int(r['inside_zone']) else 0 for r in rr])
    paths[pid] = (rr, xy, zones)
plt.rcParams.update({'font.family':'NanumGothic', 'axes.unicode_minus':False,
                     'font.size':11, 'figure.facecolor':'#fafbfd', 'axes.facecolor':'white'})
zc = {0:'#b6bcc5',1:'#287ac0',2:'#8658b8',3:'#eb9519',4:'#a98c21',5:'#da3c64',6:'#008c86'}
zl = {0:'일반 구간',1:'GPS 주행',2:'마지막 미션',3:'신호등 / GPS',5:'장애물 회피',6:'ESTOP 감지'}
pc = dict(zip(range(1,8),['#1677b8','#e88d21','#239562','#8855b5','#df4e4e','#8a704b','#cc53a3']))

def axes_style(ax):
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=.18)
    ax.set_xlabel('동쪽 거리 (m)'); ax.set_ylabel('북쪽 거리 (m)')
    ax.margins(.12)
    ax.spines[['top','right']].set_visible(False)

def zone_path(ax, xy, z, lw=3):
    segments = np.stack([xy[:-1],xy[1:]],axis=1)
    ax.add_collection(LineCollection(segments,colors=[zc[int(k)] for k in z[:-1]],linewidths=lw,alpha=.92))
    ax.scatter(xy[:,0],xy[:,1],c=[zc[int(k)] for k in z],s=5,zorder=3)
    ax.autoscale_view()

def arrows(ax, xy, color, n=4):
    for j in np.linspace(0,len(xy)-2,n+2,dtype=int)[1:-1]:
        k=min(len(xy)-1,j+5)
        ax.annotate('',xy=xy[k],xytext=xy[j],arrowprops=dict(arrowstyle='-|>',color=color,lw=1.4,mutation_scale=13))

def save(fig,name):
    fig.savefig(OUT/(name+'.png'),dpi=180,bbox_inches='tight')
    fig.savefig(OUT/(name+'.pdf'),bbox_inches='tight')
    plt.close(fig)

fig,axs=plt.subplots(1,2,figsize=(16,9),layout='constrained')
for pid,(_,xy,z) in paths.items():
    axs[0].plot(*xy.T,color=pc[pid],lw=2.5,alpha=.88,label=f'경로 {pid:02}')
    arrows(axs[0],xy,pc[pid],3)
    zone_path(axs[1],xy,z,2.8)
for ax in axs:axes_style(ax)
axs[0].set_title('경로 번호별 구분',fontweight='bold');axs[0].legend(loc='upper left',fontsize=10)
axs[1].set_title('zone별 구분 — 동일 좌표',fontweight='bold')
axs[1].legend(handles=[Line2D([0],[0],color=zc[z],lw=4,label=f'[{z}] {zl[z]}' if z else '일반 구간 (zone 0)') for z in zl],loc='upper left',fontsize=10)
rr,xy,z=paths[3]
i=np.flatnonzero(z==5)[len(np.flatnonzero(z==5))//2]
axs[1].annotate('경로 03 · 회피 [5]\nidx 337–462',xy=xy[i],xytext=(28,20),textcoords='offset points',bbox=dict(boxstyle='round,pad=.4',fc='white',ec=zc[5]),arrowprops=dict(arrowstyle='->',color=zc[5]),fontsize=11)
fig.suptitle('Yongin no parking · 저장된 전체 경로와 zone',fontsize=19,fontweight='bold')
fig.supxlabel('위·경도를 공통 원점으로 환산 · 화살표는 idx 증가 방향 · 겹치는 경로는 상세 그림에서 분리',fontsize=11)
save(fig,'overview')

fig,axs=plt.subplots(2,4,figsize=(21,13),layout='constrained')
spans=[]
for ax,(pid,(rr,xy,z)) in zip(axs.flat,paths.items()):
    zone_path(ax,xy,z,3.2);arrows(ax,xy,'#333333',3)
    ax.scatter(*xy[0],marker='o',s=55,facecolors='white',edgecolors='black',zorder=5)
    ax.scatter(*xy[-1],marker='s',s=40,c='black',zorder=5)
    axes_style(ax)
    ax.set_title(f'경로 {pid:02} · idx {rr[0]["idx"]}–{rr[-1]["idx"]}',fontweight='bold')
    starts=np.r_[0,np.flatnonzero(z[1:]!=z[:-1])+1];ends=np.r_[starts[1:]-1,len(z)-1]
    for a,b in zip(starts,ends):
        spans.append(dict(path=f'{pid:02}',zone=int(z[a]),start_idx=rr[a]['idx'],end_idx=rr[b]['idx']))
        if z[a]:
            k=(a+b)//2
            ax.annotate(f'[{z[a]}]',xy=xy[k],xytext=(5,6),textcoords='offset points',fontsize=10,color=zc[int(z[a])],fontweight='bold',bbox=dict(fc='white',ec='none',alpha=.85,pad=1))
    if pid in (3,4):
        markers = [(337,'회피 시작 337'),(463,'회피 종료 463')] if pid==3 else [(622,'감지 시작 622'),(829,'감지 종료 829')]
        marker_color = zc[5 if pid==3 else 6]
        for idx,label in markers:
            k=next(i for i,r in enumerate(rr) if int(r['idx'])==idx)
            ax.annotate(label,xy=xy[k],xytext=(8,-28 if idx in (337,622) else 20),textcoords='offset points',fontsize=9,arrowprops=dict(arrowstyle='->',color=marker_color),bbox=dict(fc='white',ec=marker_color,alpha=.95,boxstyle='round,pad=.3'))
ax=axs.flat[-1];ax.axis('off')
ax.text(.03,.96,'표시 기준',transform=ax.transAxes,fontsize=18,fontweight='bold',va='top')
for j,z in enumerate(zl):
    ax.plot([.05,.19],[.84-j*.085]*2,color=zc[z],lw=5,transform=ax.transAxes)
    ax.text(.24,.84-j*.085, f'[{z}] {zl[z]}' if z else '일반 구간 (zone 0)',transform=ax.transAxes,va='center',fontsize=13)
ax.text(.03,.25,'○ 시작   ■ 끝\n화살표: idx 증가 방향\n\n경로별 축 범위는 다릅니다.\nstate 번호와 zone 번호는 별개입니다.\n색상은 zone_id / inside_zone 기준입니다.',transform=ax.transAxes,va='top',fontsize=12,linespacing=1.65)
fig.suptitle('경로 01–07 상세 · zone 색상 및 주행 방향',fontsize=21,fontweight='bold')
save(fig,'routes_by_zone')
with (OUT/'zone_ranges.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=['path','zone','start_idx','end_idx']);w.writeheader();w.writerows(spans)
(OUT/'metadata.json').write_text(json.dumps(dict(source=str(SOURCE),sha256=hashlib.sha256(data).hexdigest(),rows=len(rows),origin_latlon=[lat0,lon0]),indent=2))
print(OUT/'overview.png');print(OUT/'routes_by_zone.png')
