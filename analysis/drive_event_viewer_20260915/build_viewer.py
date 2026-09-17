"""Build an offline viewer from recorded GPS, snapshots and state transitions.

No controller replay and no vehicle connection. Dashed geometry is explanatory,
not a recovery of the unrecorded full nav_msgs/Path or surveyed stop line.
"""
import ast
import bisect
import csv
import json
import math
import re
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
KST = ZoneInfo('Asia/Seoul')
SPECS = [
    ('063951_118844', '신호등 인식 → 정지', 1789335628.656, -16, 90, 'mgm_node_40331_1789335591233.log'),
    ('224009_892949', 'A·B·C 경로 132점 생성', 1789393247.297, -12, 25, 'python3_99685_1789393210376.log'),
    ('225420_885638', 'A·B·C 경로 107점 생성', 1789394145.551, -12, 35, 'python3_102177_1789394061546.log'),
    ('231054_732541', 'fixed_goals 곡선 생성', 1789395126.627, -12, 24, 'python3_104611_1789395055389.log'),
]
NAMES = {
    'top': ['ENABLE', 'DRIVE', 'FINISH'],
    'navigation': ['LINE', 'GPS_BACKUP', 'GPS_ONLY_NAV'],
    'avoidance': ['INACTIVE', 'AVOID_ACTIVE', 'CLEAR_CONFIRM', 'GPS_RETURN'],
    'signal': ['SIGNAL_IDLE', 'RED_DETECTED', 'APPROACH_STOP_LINE', 'STOPPED_WAIT'],
    'safety': ['NORMAL', 'AUTO_ESTOP', 'REVERSE_RECOVERY', 'SAFE_STOP'],
    'speed_owner': ['NAVIGATION', 'AVOIDANCE', 'TRAFFIC', 'MISSION', 'SAFETY', 'FINISH'],
    'reference_source': ['LANE', 'GPS', 'AVOID', 'PARKING', 'ESCAPE'],
}

def read(path):
    return list(csv.DictReader(path.open()))

def clock(t):
    return datetime.fromtimestamp(t, KST).strftime('%H:%M:%S.%f')[:-3]

def transform(p, xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [xy[0] + c*p[0] - s*p[1], xy[1] + s*p[0] + c*p[1]]

result = []
for suffix, title, event, before, after, logname in SPECS:
    run = ROOT / 'drive_logs' / ('v2_20260914_' + suffix)
    gps = read(run / 'lateral.csv')
    inp = read(ROOT/'analysis/run_20260914_063951_diagnosis/inputs.csv' if suffix.startswith('0639') else OUT/(suffix+'_inputs.csv'))
    transitions = read(run/'transitions.csv')
    times = [int(r['event_time_ns'])/1e9 for r in inp]
    tt = [int(r['tick']) for r in transitions]
    assert all(b > a for a, b in zip(times, times[1:]))
    assert tt == sorted(tt) and tt[-1] < len(inp)
    lat0, lon0 = float(gps[0]['lat']), float(gps[0]['lon'])
    lon_scale = 111320*math.cos(math.radians(lat0))
    def xy(g):
        return [(float(g['lon'])-lon0)*lon_scale, (float(g['lat'])-lat0)*111320]
    frames = []
    traffic_samples=[]
    if suffix.startswith('0639'):
        for line in (ROOT/'log_v2/ros/python3_40329_1789335593588.log').read_text().splitlines():
            tm=re.search(r'\[(\d+\.\d+)\]',line)
            det=re.search(r'\| stopline=(0|1|off)(?: |$)',line)
            if tm and det and 'frame=' in line:
                traffic_samples.append((float(tm[1]),int(det[1]) if det[1]!='off' else None))
    for g in gps:
        t = float(g['stamp_s'])
        if not event+before <= t <= event+after:
            continue
        tick = bisect.bisect_right(times, t)-1
        if tick < 0:
            continue
        a = inp[tick]
        tr = transitions[bisect.bisect_right(tt, tick)-1]
        pos, yaw = xy(g), math.radians(float(g['heading_deg']))
        age = t-times[tick]
        assert 0 <= age < .1
        n = int(a['avoid_path_n'])
        point = transform([float(a['avoid_x']),float(a['avoid_y'])],pos,yaw) if n else None
        frames.append(dict(t=t,time=clock(t),xy=pos,yaw=yaw,lat=float(g['lat']),lon=float(g['lon']),
            quality=int(g['quality']),heading_source=g['heading_src'],route=g['route_id'],idx=int(g['idx']),
            station=float(g['station_m']),tick=tick,age_ms=round(age*1000,2),
            v=float(a['vehicle_speed']) if a['vehicle_speed_valid']=='1' else None,
            v_transition=float(tr['v_ref']),red=int(a['red']),green=int(a['green']),
            n=n,point=point,reference_age=float(a['avoid_age_s']),reference_timeout=float(a['avoid_timeout_s']),
            state={k:NAMES[k][int(tr[k])] for k in NAMES},stop_reasons=int(tr['stop_reasons']),
            ref_valid=int(tr['ref_valid']),ref_fresh=int(tr['ref_fresh'])))
        frame=frames[-1]
        if traffic_samples:
            j=bisect.bisect_right([v[0] for v in traffic_samples],t)-1
            frame['stopline_detected']=traffic_samples[j][1] if j>=0 else None
            frame['stopline_age_s']=round(t-traffic_samples[j][0],3) if j>=0 else None
        else:
            frame['stopline_detected']=int(a['stopline']) if 'stopline' in a else None
            frame['stopline_age_s']=None
    assert frames
    ft = [f['t'] for f in frames]
    def nearest(t):
        return min(range(len(frames)),key=lambda i:abs(ft[i]-t))
    events=[]
    for tr in transitions:
        t=times[int(tr['tick'])]
        if ft[0] <= t <= ft[-1]:
            desc=' / '.join(NAMES[k][int(tr[k])] for k in ['signal','avoidance','safety'])
            events.append(dict(t=t,index=nearest(t),text=desc,type='state',time=clock(t)))
    log=(ROOT/'log_v2/ros'/logname).read_text()
    anchors=None
    for number,line in enumerate(log.splitlines(),1):
        match=re.search(r'\[(\d+\.\d+)\]',line)
        if not match:
            continue
        t=float(match[1])
        if not ft[0] <= t <= ft[-1]:
            continue
        if 'A-B-C planner:' in line or 'avoid planner:' in line:
            text=line.split('planner:',1)[1].strip()
            events.append(dict(t=t,index=nearest(t),text=text.split(';')[0],type='planner',time=clock(t),line=number))
            m=re.search(r'anchors=(\(.*?\)); points=',line)
            if m and anchors is None:
                local=ast.literal_eval(m[1]); f=frames[nearest(t)]
                anchors=dict(t=t,xy=[transform(p,f['xy'],f['yaw']) for p in local],local=local)
    stopline=None
    if suffix.startswith('0639'):
        # First log sample with latched distance. This is a controller estimate,
        # including its configured distance seed; it is NOT surveyed geometry.
        t=1789335626.836313512
        f=frames[nearest(t)]
        centre=transform([.21,0],f['xy'],f['yaw'])
        stopline=dict(t=t,xy=[transform([0,-1.5],centre,f['yaw']),transform([0,1.5],centre,f['yaw'])],
                      note='06:40:26.836 제어기 잔여거리 0.21m로 투영. 폭 3m는 표시용이며 실측 정지선이 아님.')
    routes=[]
    for rid in sorted({f['route'] for f in frames}):
        path=ROOT/'src/stack_gps/waypoints'/('waypoints_halla_reference_path_'+rid+'.csv')
        if path.exists():
            routes.append(dict(id=rid,xy=[xy(g) for g in read(path)]))
    modefile=run/'avoid_planner_mode.txt'
    result.append(dict(id=suffix,title=title,mode=modefile.read_text().strip() if modefile.exists() else '당시 GPS cubic',
        frames=frames,events=sorted(events,key=lambda e:e['t']),anchors=anchors,stopline=stopline,routes=routes,
        selected=min(bisect.bisect_left(ft,event),len(ft)-1),origin=[lat0,lon0],log='../../log_v2/ros/'+logname,
        note=('GPS 기준 투영. 당시 고정에 사용한 좌표는 CAN x/y이므로 절대 경로의 재생이 아님.' if suffix.startswith('2240') else
              'GPS 기준 투영. 당시 고정에 사용한 좌표는 속도·헤딩 적분 좌표이므로 절대 경로의 재생이 아님.' if suffix.startswith('2254') else
              '전체 회피 곡선 원본 미기록. 저장된 회피 제어점을 GPS 위치·헤딩으로 지도에 투영.')))

(OUT/'data.json').write_text(json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':')))
template=(OUT/'viewer_template.html').read_text()
(OUT/'index.html').write_text(template.replace('__DATA__',json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(',',':'))))
print('Built',[(r['id'],len(r['frames']),len(r['events'])) for r in result])

# Shareable overview: actual GPS trajectories, recorded point projections,
# explicitly dashed approximate geometry. No fabricated full reference curve.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font=next((f.name for f in font_manager.fontManager.ttflist if f.name=='Noto Sans CJK JP'),None)
if font:
    plt.rcParams['font.family']=font
plt.rcParams['axes.unicode_minus']=False
fig, axes=plt.subplots(2,2,figsize=(15,12),layout='constrained')
colors={'NORMAL':'#1aaf9b','AUTO_ESTOP':'#e85555','SAFE_STOP':'#9b59b6','REVERSE_RECOVERY':'#e89b31'}
for ax,r in zip(axes.flat,result):
    fs=r['frames']; f=fs[r['selected']]
    for route in r['routes']:
        ax.plot(*zip(*route['xy']),color='#aeb9c5',lw=1,label='GPS waypoint (현재 파일)')
    ax.plot(*zip(*(f['xy'] for f in fs)),color='#d4dce4',lw=1)
    for state,color in colors.items():
        pts=[f['xy'] for f in fs if f['state']['safety']==state]
        if pts: ax.scatter(*zip(*pts),s=9,c=color,label=state)
    pts=[f['point'] for f in fs if f['point'] and f['reference_age']<=f['reference_timeout']]
    if pts:ax.scatter(*zip(*pts),s=7,c='#e14ce0',alpha=.6,label='회피 제어점 GPS 투영')
    if r['anchors']:
        ax.plot(*zip(*r['anchors']['xy']),'--o',c='#9863cf',label='A·B·C 연결 개념선')
        for label,p in zip('ABC',r['anchors']['xy']):ax.annotate(label,p,xytext=(5,5),textcoords='offset points')
    if r['stopline']:ax.plot(*zip(*r['stopline']['xy']),'--',c='#dc4747',lw=3,label='정지선 추정 위치')
    ax.scatter(*f['xy'],s=100,c='#182c45',marker='*',zorder=5,label=f['time']+' 차량')
    pts=[f['xy'] for f in fs]+([f['point'] for f in fs if f['point']])
    for d in ('anchors','stopline'):
        if r[d]:pts+=r[d]['xy']
    xx,yy=zip(*pts);ax.set_xlim(min(xx)-3,max(xx)+3);ax.set_ylim(min(yy)-3,max(yy)+3)
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('East [m]');ax.set_ylabel('North [m]')
    ax.set_title(r['title']+' | '+f['time']+'\n'+f['state']['signal']+' / '+f['state']['avoidance']+' / '+f['state']['safety'],fontsize=10)
    ax.legend(fontsize=7,loc='best')
fig.suptitle('2026-09-14 주행 기록 · 차량 위치와 상태\n점선은 추정/개념선. 전체 회피 곡선 원본은 미기록.',fontsize=15)
fig.savefig(OUT/'overview.png',dpi=150)
print('Saved overview.png')
