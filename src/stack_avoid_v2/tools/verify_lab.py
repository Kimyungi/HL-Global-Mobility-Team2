#!/usr/bin/env python3
"""Check embedded C++ recordings and exercise the offline UI in headless Chrome."""
import argparse
import html
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]

HARNESS = r'''
<script>
setTimeout(() => {
  let checks=0;
  const $=id=>document.getElementById(id);
  const ok=(v,msg)=>{if(!v)throw Error(msg);checks++;};
  const click=id=>$(id).click();
  const tick=t=>{const cb=window.labRAF;window.labRAF=null;cb(t);};
  try {
    const lab=window.V2Lab;
    ok(lab?.ready,'application started');
    ok(document.querySelectorAll('.scenario').length===lab.data.scenarios.length,'scenario controls');
    for(const n of [2,3,0]){
      click(n?'cases'+n:'casesAll');
      ok([...document.querySelectorAll('.scenario')].filter(b=>!b.hidden).length===(n?13:26),'vehicle count filter');
      if(n)ok(lab.scene().boxes.length===n,'filter selects matching scene');
    }
    for(let i=0;i<lab.data.scenarios.length;i++){
      document.querySelectorAll('.scenario')[i].click();
      ok(lab.ui.scenario===i,'scenario click');
      for(const at of [0,Math.floor(lab.scene().frames.length/2),lab.scene().frames.length-1]){
        lab.setFrame(at);const f=lab.frame();
        ok(!!$('selectedPath')===f.valid,'path validity');
        ok(!!$('referencePoint')===f.valid,'target validity');
        ok(!!$('holdNotice')===(!f.valid&&f.phase===4),'HOLD notice');
        ok($('pointCount').textContent===(f.valid?f.path.length:0)+'개','point count');
        ok($('sceneTitle').textContent===lab.scene().title,'scene title');
        ok($('actualSpeed').textContent===f.ego[3].toFixed(2)+'m/s','measured speed display');
        ok($('targetSpeed').textContent===(f.valid?f.speedLimit.toFixed(2)+'m/s':'HOLD'),'requested speed display');
        ok(!!$('curbEdges'),'hard road edge layer');
        ok($('laneDimension').textContent.includes('3.0m'),'single lane dimension');
      }
    }
    const replay=lab.data.scenarios.findIndex(s=>s.frames.length>6&&s.frames[0].valid);
    ok(replay>=0,'multi-frame recording available');
    lab.setScenario(replay);click('next');ok(lab.ui.frame===1,'next');
    click('previous');ok(lab.ui.frame===0,'previous');
    click('last');ok(lab.ui.frame===lab.scene().frames.length-1,'recording endpoint');
    ok(lab.frame().complete===(lab.scene().outcome==='complete'),'endpoint preserves actual completion result');
    click('first');ok(lab.ui.frame===0,'first');
    const seek=Math.min(10,lab.scene().frames.length-4);
    $('timeline').value=seek;$('timeline').dispatchEvent(new Event('input'));
    ok(lab.ui.frame===seek,'timeline seek');
    document.dispatchEvent(new KeyboardEvent('keydown',{code:'ArrowRight',bubbles:true}));
    ok(lab.ui.frame===seek+1,'keyboard seek');
    click('play');ok(lab.ui.playing&&$('play').textContent==='일시정지','play');
    tick(100);tick(200);ok(lab.ui.frame>seek+1,'animation advances recorded frames');
    click('play');ok(!lab.ui.playing,'pause');
    $('speed').value=4;tick(300);click('first');click('play');tick(400);
    ok(lab.ui.frame===4,'4x replay speed');click('play');
    click('last');click('play');ok(lab.ui.frame===0&&lab.ui.playing,'replay from end');
    click('play');
    for(let step=0;step<5;step++){
      document.querySelector(`[data-step="${step}"]`).click();
      ok(lab.ui.step===step,'step control');
      ok(!!$('selectedPath')===(step>=3),'step path visibility');
      ok(!!$('referencePoint')===(step===4),'step reference visibility');
      if(step===1||step===2)ok(!!$('searchEdges'),'search edge layer');
      ok(!!$('wallMidpoints')===(step>=1),'midpoint layer follows step');
    }
    click('allSteps');click('showGrid');ok(!$('unknownCells'),'grid toggle');
    click('showGrid');ok(!!$('unknownCells'),'grid restore');
    click('showEnvelope');ok(!$('bodyEnvelope'),'envelope toggle');
    click('showEnvelope');click('showTrace');ok(!!$('searchEdges'),'trace toggle');
    click('showTrace');click('overview');ok(!lab.ui.focus,'overview');
    click('focus');ok(lab.ui.focus,'focus');
    const map=$('map'),path=$('selectedPath'),p=path.getPointAtLength(30);
    const at=p.matrixTransform(map.getScreenCTM());
    map.dispatchEvent(new MouseEvent('mousemove',{clientX:at.x,clientY:at.y,bubbles:true}));
    ok(!$('tooltip').hidden&&$('tooltip').textContent.includes('경로 점'),'point tooltip');
    map.dispatchEvent(new MouseEvent('mouseleave'));ok($('tooltip').hidden,'tooltip hides');
    lab.setScenario(lab.data.scenarios.findIndex(s=>s.id==='close_braking_2'));ok($('play').disabled,'single-frame HOLD cannot play');
    lab.setScenario(0);lab.setStep(-1);click('cases2');click('overview');
    ok(document.documentElement.scrollWidth<=innerWidth,'viewport has no horizontal overflow');
    ok(window.labErrors.length===0,'no browser script errors');
    const result=document.createElement('pre');result.id='lab-test-result';
    result.textContent=JSON.stringify({ok:true,checks,width:innerWidth});document.body.append(result);
    if(parent!==window)parent.postMessage(result.textContent,'*');
  } catch(error) {
    const result=document.createElement('pre');result.id='lab-test-result';
    result.textContent=JSON.stringify({ok:false,checks,error:String(error),errors:window.labErrors});
    document.body.append(result);
    if(parent!==window)parent.postMessage(result.textContent,'*');
  }
}, 200);
</script>
'''


def verify(source, browser, screenshot_dir):
    page = source.read_text()
    data = json.loads(re.search(r'<script id="replay-data" type="application/json">(.*?)</script>', page, re.S)[1])
    assert data['schema'] == 4 and len(data['scenarios']) == 26
    assert data['algorithm'] == 'wall_midpoint_corridor'
    model = data['model']
    assert model['cruiseSpeed'] == 1 and data['stepSeconds'] == .1
    assert model['adaptiveSpeed'] and model['crawlSpeed'] == .2
    assert model['curbPlacement'] == 'road_edges'
    assert math.isclose(model['front'] + model['rear'], .85) and model['width'] == .62
    assert not re.search(r'<(?:script|link|img)[^>]+(?:src|href)=["\']https?://', page)
    total = 0
    for scene in data['scenarios']:
        assert scene['laneWidth'] == 3
        # Both curb polylines are parallel offsets of each GPS segment. Check
        # perpendicular distances independently of the fixture's miter formula.
        boundary = scene['boundary']; side = len(boundary)//2
        right, left = boundary[:side], list(reversed(boundary[side:]))
        for i, (a, b) in enumerate(zip(scene['center'], scene['center'][1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]; length = math.hypot(dx, dy)
            for edge, sign in ((right, -1), (left, 1)):
                for p in edge[i+1:i+3]:
                    distance = (dx*(p[1]-a[1])-dy*(p[0]-a[0]))/length
                    assert abs(distance-sign*1.5) < .0001, (scene['id'], i, distance)
        assert len(scene['boxes']) in (2, 3)
        assert len(scene['placements']) == len(scene['boxes'])
        assert scene['curbs'] and all(len(edge) == 4 for edge in scene['curbs'])
        assert all(math.isclose(b[2], model['front'] + model['rear']) and b[3] == model['width'] for b in scene['boxes'])
        assert not scene['audit']['vehicleContact'] and not scene['audit']['curbContact'], scene['id']
        for grid in scene['maps']:
            assert len(grid['rle']) % 2 == 0
            assert sum(grid['rle'][::2]) == grid['width'] * grid['height']
            assert all(v in range(4) for v in grid['rle'][1::2])
        for index, frame in enumerate(scene['frames']):
            total += 1
            assert abs(frame['time'] - index * .1) < 1e-5
            assert .1999 <= frame['ego'][3] <= 1.0001
            assert all(.1999 <= p[3] <= 1.0001 for p in frame['path'])
            assert not frame['vehicleContact'] and not frame['curbContact']
            assert not frame['maneuver']
            sizes = [len(frame[k]) for k in ('wallLeft', 'wallRight', 'midpoints', 'corridor')]
            assert len(set(sizes)) == 1
            for left, right, mid in zip(frame['wallLeft'], frame['wallRight'], frame['midpoints']):
                assert math.dist(mid, [(a+b)/2 for a,b in zip(left,right)]) < .0001
            assert 0 <= frame['map'] < len(scene['maps'])
            assert len(frame['trace']) <= data['model']['traceLimit']
            assert sum(frame['counts']) >= len(frame['trace'])
            assert bool(frame['path']) == frame['valid']
            assert (frame['reference'] is not None) == frame['valid']
            assert all(math.isfinite(x) for p in frame['path'] for x in p)
            if frame['valid']:
                assert frame['phase'] in (5, 6) and not frame['gpsFollow']
                assert len(frame['corridor']) >= 8
                path = frame['path']
                assert .2 <= frame['speedLimit'] <= 1
                assert math.dist(path[0][:2], frame['ego'][:2]) < .0001
                assert abs(path[0][3]-frame['ego'][3]) < .0001
                for a,b in zip(path,path[1:]):
                    ds = math.dist(a[:2], b[:2]); dt = 2*ds/(a[3]+b[3])
                    accel = (b[3]**2-a[3]**2)/(2*ds)
                    assert -model['deceleration']-.002 <= accel <= model['acceleration']+.002
                    assert abs(b[4]-a[4]) <= model['steerRate']*dt+.0001
                    assert b[3]**2*abs(math.tan(b[4])/model['wheelbase']) <= .6001
                if index+1 < len(scene['frames']):
                    remaining = .1
                    for a,b in zip(path,path[1:]):
                        ds=math.dist(a[:2],b[:2]); dt=2*ds/(a[3]+b[3])
                        if remaining <= dt:
                            accel=(b[3]-a[3])/dt
                            ratio=(a[3]*remaining+.5*accel*remaining**2)/ds
                            expected=[a[k]+ratio*(b[k]-a[k]) for k in (0,1)]
                            actual=scene['frames'][index+1]['ego']
                            assert math.dist(expected,actual[:2]) < .0002
                            assert abs(actual[3]-(a[3]+accel*remaining)) < .0001
                            break
                        remaining-=dt
                target = path.index(frame['reference'])
                arc = sum(math.dist(a[:2], b[:2]) for a,b in zip(path[:target], path[1:target+1]))
                # Export rounding can shift a threshold comparison by a few micrometres.
                assert .9999 <= arc <= 1.0501, (scene['id'], index, arc)
                assert frame['ms'] <= data['model']['budget']
        last = scene['frames'][-1]
        if scene['id'].startswith('zone_gate_'):
            assert scene['frames'][0]['phase'] == 0 and not scene['frames'][0]['perception']
            assert any(f['perception'] and f['detected'] for f in scene['frames'])
        assert (scene['outcome'] == 'complete') == last['complete']
        if scene['outcome'] == 'hold':
            assert not last['valid']
        print(f"{scene['id']}: {len(scene['frames'])} frames, {scene['outcome']}")
    print(f'Data validation: {total} real planner frames passed')
    setup = '<script>window.labErrors=[];addEventListener("error",e=>labErrors.push(e.message));window.requestAnimationFrame=cb=>{window.labRAF=cb;return 1;};</script>'
    test_page = page.replace('<head>', '<head>'+setup, 1).replace('</body>', HARNESS+'</body>')
    screenshot_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='avoid_v2_verify_') as tmp:
        test_file = Path(tmp)/'test.html'
        test_file.write_text(test_page)
        for width in (1440, 390):
            target = test_file
            if width < 500:
                # Some Chrome versions clamp the initial top-level viewport to 500px.
                # A fixed-width iframe gives the application the requested CSS viewport.
                target = Path(tmp)/'narrow.html'
                target.write_text('<!doctype html><meta charset="utf-8">'
                    '<style>body{margin:0;background:#0b111b}iframe{border:0;display:block}</style>'
                    '<script>addEventListener("message",e=>{const p=document.createElement("pre");'
                    'p.id="lab-test-result";p.textContent=e.data;document.body.append(p);});</script>'
                    f'<iframe src="test.html" width="{width}" height="2400"></iframe>')
            result = subprocess.run([
                browser, '--headless', '--no-sandbox', '--disable-gpu',
                '--disable-background-networking', '--no-first-run',
                f'--user-data-dir={tmp}/profile-{width}', f'--window-size={width},1400',
                '--virtual-time-budget=1500', '--dump-dom',
                f'--screenshot={screenshot_dir}/avoid-v2-{width}.png', target.as_uri(),
            ], capture_output=True, text=True, timeout=45)
            assert result.returncode == 0, result.stderr[-1500:]
            match = re.search(r'<pre id="lab-test-result">(.*?)</pre>', result.stdout, re.S)
            assert match, 'Browser did not complete test harness: '+result.stderr[-1500:]
            report = json.loads(html.unescape(match[1]))
            assert report['ok'], report
            assert report['width'] == width, report
            print('Browser:', report)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('html', type=Path, nargs='?', default=ROOT/'docs/avoid_v2_path_lab.html')
    parser.add_argument('--browser', default=shutil.which('google-chrome') or shutil.which('chromium'))
    parser.add_argument('--screenshots', type=Path, default=Path('/tmp/avoid_v2_browser_check'))
    args = parser.parse_args()
    if not args.browser:
        parser.error('Chrome or Chromium is required for UI verification')
    verify(args.html.resolve(), args.browser, args.screenshots)
