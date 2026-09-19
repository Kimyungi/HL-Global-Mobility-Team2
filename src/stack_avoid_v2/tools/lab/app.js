"use strict";
(() => {
  const data = JSON.parse(document.getElementById("replay-data").textContent);
  const $ = id => document.getElementById(id);
  const model = data.model;
  const ui = {scenario:0, frame:0, step:-1, focus:false, playing:false, lastTime:0, accumulator:0, count:2};
  const phaseNames = ["GPS_OUTSIDE", "PASS", "REJOIN", "GPS_FOLLOW", "HOLD", "COMPLETE", "WALL_FOLLOW"];
  const phaseKo = ["zone 밖", "연속 추월", "GPS 복귀", "GPS 추종", "진행 보류", "출구 통과 확인", "벽 사이 중앙 경로"];
  const stepText = [
    ["01 · LiDAR 자유 공간과 벽", "zone 진입 후 관측을 시작합니다. 장애물과 연석을 같은 점유 경계로 취급하며, 미관측 공간은 통과하지 않습니다. GPS는 진행 방향과 출구를 정합니다."],
    ["02 · 양쪽 벽의 중간점", "GPS에 수직인 단면마다 관측된 자유 통로를 찾습니다. 차량 폭과 여유가 들어가고 앞뒤로 연결되는 통로를 선택해, 양쪽 경계 중간에 점을 찍습니다. 노란 가로선과 보라색 점이 이 과정입니다."],
    ["03 · 연결과 차체 검사", "중간점 연결선을 선택한 통로 안에서 부드럽게 만듭니다. 앞범퍼가 먼저 벽에 도달하는 것을 고려한 연결선도 비교합니다. 통로 폭·곡률·조향 변화량에 따라 0.2~1.0m/s 속도를 정하고 미리 감속합니다. 차체·조향 지연·횡가속뿐 아니라 경로 표본 사이의 제동 공간도 검사합니다. 한쪽 통로가 실패하면 반대쪽 후보도 비교합니다."],
    ["04 · 벽 사이 레퍼런스 패스", "보라색은 벽 중간점에서 만든 안내선, 청록색은 차량 운동 모델로 생성하고 검증한 레퍼런스 패스입니다. 새 관측으로 경로를 재검사하며, 안전한 기존 경로의 속도 계획을 유지해 매 프레임 감속이 끊기지 않게 합니다."],
    ["05 · 경로를 따라 약 1m 앞", "누적 경로 길이가 1m 이상인 첫 표본을 목표점으로 출력합니다. 표본 간격은 약 5cm이며 접선과 곡률을 함께 제공합니다."]
  ];
  const reasonText = {
    "outside obstacle zone; GPS owns reference":"zone 밖에서는 기존 GPS가 경로를 담당합니다.",
    "obstacle zone membership unavailable":"zone 정보가 유효하지 않아 HOLD합니다.",
    "wall corridor speed profile revalidated":"새 관측으로 기존 경로의 차체·제동 공간을 다시 검사하고 감속 계획을 이어갑니다.",
    "wall midpoint corridor certified":"벽 사이 중간점을 연결하고 차체·조향·표본 사이 제동 공간을 검사한 경로입니다.",
    "no connected observed wall corridor":"차량이 들어갈 폭의 관측 통로를 앞뒤로 연결하지 못했습니다.",
    "wall midline has no certified forward rollout":"벽 사이 안내선은 계산했지만, 필요한 길이의 차체·조향·제동 검증 경로를 만들지 못했습니다.",
    "vehicle heading opposes GPS direction":"차량 방향이 GPS 진행 방향과 반대여서 HOLD합니다.",
    "wall corridor processing deadline exceeded":"벽 사이 경로 계산 예산을 초과했습니다.",
    "current braking envelope blocked or unknown":"현재 조향에서 제동할 차체 영역이 막혔거나 미관측입니다.",
    "planner processing deadline exceeded":"코어 처리 기한을 초과해 계산 결과를 무효화했습니다.",
    "intermediate braking envelope blocked":"경로 중간의 제동 영역이 막혀 후보를 버렸습니다.",
    "terminal braking envelope blocked":"경로 끝의 제동 공간이 부족합니다.",
    "insufficient planning horizon":"목표점과 정지 공간을 검증할 경로 길이가 부족합니다.",
    "invalid or repeated input":"유효하지 않거나 반복된 입력이므로 새 경로를 출력하지 않습니다."
  };
  const scene = () => data.scenarios[ui.scenario];
  const frame = () => scene().frames[ui.frame];
  const n = (v, digits=2) => Number.isFinite(v) ? v.toFixed(digits) : "—";
  const esc = s => String(s).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  const degrees = a => a*180/Math.PI;
  const wrap = a => Math.atan2(Math.sin(a),Math.cos(a));
  let transform = null;

  function decodeMap(map) {
    if (map.runs) return map.runs;
    const cells = new Uint8Array(map.width*map.height);
    let at=0;
    for(let i=0;i<map.rle.length;i+=2){cells.fill(map.rle[i+1],at,at+map.rle[i]);at+=map.rle[i];}
    if(at!==cells.length) throw new Error("격자 기록 길이가 일치하지 않습니다.");
    const runs=[];
    for(let y=0;y<map.height;y++){
      for(let x=0;x<map.width;){
        const v=cells[y*map.width+x];let end=x+1;
        while(end<map.width&&cells[y*map.width+end]===v)end++;
        if(v===0||v===2)runs.push([x,y,end-x,v]);
        x=end;
      }
    }
    map.runs=runs;return runs;
  }

  function mapper() {
    const s=scene(), f=frame(), W=960,H=440;
    let minx,maxx,miny,maxy;
    if(ui.focus){minx=f.ego[0]-2.5;maxx=f.ego[0]+8.5;miny=f.ego[1]-3.7;maxy=f.ego[1]+3.7;}
    else {minx=Math.min(...s.boundary.map(p=>p[0]))-.6;maxx=Math.max(...s.boundary.map(p=>p[0]))+.6;miny=Math.min(...s.boundary.map(p=>p[1]))-.8;maxy=Math.max(...s.boundary.map(p=>p[1]))+.8;}
    const scale=Math.min((W-60)/(maxx-minx),(H-70)/(maxy-miny));
    const ox=(W-(maxx-minx)*scale)/2,oy=(H-(maxy-miny)*scale)/2;
    return {W,H,scale,minx,maxx,miny,maxy,point:p=>[ox+(p[0]-minx)*scale,H-oy-(p[1]-miny)*scale]};
  }
  function drawMap() {
    const s=scene(), f=frame(), m=mapper();transform=m;
    const p=m.point, path=points=>points.map((q,i)=>`${i?"L":"M"}${p(q).map(v=>n(v,2)).join(",")}`).join(" ");
    const poly=points=>points.map(q=>p(q).map(v=>n(v,2)).join(",")).join(" ");
    const map=s.maps[f.map], parts=[];
    parts.push(`<defs><pattern id="hatch" width="9" height="9" patternUnits="userSpaceOnUse"><rect width="9" height="9" fill="#252c39"/><path d="M-2,2L2,-2 M0,9L9,0 M7,11L11,7" stroke="#45505c" stroke-width="1"/></pattern><clipPath id="roadClip"><polygon points="${poly(s.boundary)}"/></clipPath></defs>`);
    for(let x=Math.ceil(m.minx);x<=m.maxx;x++){const a=p([x,m.miny]),b=p([x,m.maxy]);parts.push(`<path d="M${a}L${b}" stroke="#172b3c" stroke-width=".7"/>`);if(x%2===0)parts.push(`<text x="${a[0]}" y="423" text-anchor="middle" class="svg-small">${x}m</text>`);}
    for(let y=Math.ceil(m.miny);y<=m.maxy;y++){const a=p([m.minx,y]),b=p([m.maxx,y]);parts.push(`<path d="M${a}L${b}" stroke="#172b3c" stroke-width=".7"/>`);}
    parts.push(`<polygon points="${poly(s.boundary)}" fill="#1a3041" stroke="#66829a" stroke-width="1.5"/>`);
    parts.push('<g id="curbEdges" fill="none" stroke="#edc272" stroke-width="4">');
    for(const e of s.curbs||[]){parts.push(`<path d="${path([[e[0],e[1]],[e[2],e[3]]])}"/>`);}
    parts.push('</g>');
    if($("showGrid").checked){
      let unknown="",occupied="";
      for(const [x,y,width,v] of decodeMap(map)){
        const pos=p([map.origin[0]+x*map.res,map.origin[1]+(y+1)*map.res]);
        const d=`M${n(pos[0])},${n(pos[1])}h${n(width*map.res*m.scale)}v${n(map.res*m.scale+.1)}h-${n(width*map.res*m.scale)}Z`;
        if(v===0)unknown+=d;else occupied+=d;
      }
      parts.push(`<g clip-path="url(#roadClip)"><path id="unknownCells" d="${unknown}" fill="url(#hatch)"/><path d="${occupied}" fill="#cc7452" opacity=".65"/></g>`);
    }
    parts.push(`<path id="gpsCenterline" d="${path(s.center)}" fill="none" stroke="#93a8bf" stroke-width="1.5" stroke-dasharray="7 6" opacity=".8"/>`);
    const exit=s.exitPoint;
    let near=0;for(let i=0;i<s.center.length;i++)if(Math.hypot(s.center[i][0]-exit[0],s.center[i][1]-exit[1])<Math.hypot(s.center[near][0]-exit[0],s.center[near][1]-exit[1]))near=i;
    const before=s.center[Math.max(0,near-1)],after=s.center[Math.min(s.center.length-1,near+1)];
    const half=s.laneWidth/2;
    const yaw=Math.atan2(after[1]-before[1],after[0]-before[0]),a=p([exit[0]-Math.sin(yaw)*half,exit[1]+Math.cos(yaw)*half]),b=p([exit[0]+Math.sin(yaw)*half,exit[1]-Math.cos(yaw)*half]);
    parts.push(`<path d="M${a}L${b}" stroke="#78c4a3" stroke-width="2" stroke-dasharray="4 5"/><text x="${a[0]+7}" y="${a[1]-8}" class="svg-label">출구</text>`);
    const q=s.center[8],r=s.center[9],theta=Math.atan2(r[1]-q[1],r[0]-q[0]);
    const wa=p([q[0]-Math.sin(theta)*half,q[1]+Math.cos(theta)*half]),wb=p([q[0]+Math.sin(theta)*half,q[1]-Math.cos(theta)*half]);
    parts.push(`<g id="laneDimension"><path d="M${wa}L${wb}" stroke="#edc272" stroke-width="1" stroke-dasharray="3 3"/><text x="${wa[0]-8}" y="${wa[1]-12}" text-anchor="middle" fill="#edc272" font-size="12">1차로 ${n(s.laneWidth,1)}m</text></g>`);
    if($("showHistory").checked&&ui.frame>0)parts.push(`<path d="${path(s.frames.slice(0,ui.frame+1).map(q=>q.ego))}" fill="none" stroke="#729cc0" stroke-width="2.3" opacity=".8"/>`);
    s.boxes.forEach((box,i)=>{
      const at=p([box[0]-box[2]/2,box[1]+box[3]/2]);
      const side=s.placements?.[i];
      const label=`차량 ${i+1}${side?` · ${side[1]>0?'좌':'우'} ${n(Math.abs(side[1]),2)}m`:''}`;
      parts.push(`<g class="object"><rect x="${at[0]}" y="${at[1]}" width="${box[2]*m.scale}" height="${box[3]*m.scale}" rx="3" fill="#e99556" stroke="#ffcda1" stroke-width="1.5"/><text x="${at[0]+box[2]*m.scale/2}" y="${at[1]-8}" text-anchor="middle" class="svg-label">${label}</text></g>`);
      if(s.expired&&ui.frame>0)parts.push(`<text x="${at[0]-8}" y="${at[1]+box[3]*m.scale+16}" class="svg-small">점유 기억 유지</text>`);
    });
    if(ui.step===-1||ui.step>=1){
      parts.push('<g id="wallMidpoints">');
      for(let i=0;i<(f.midpoints||[]).length;i++){
        const mid=p(f.midpoints[i]);
        if(i%2===0)parts.push(`<path d="M${p(f.wallLeft[i])}L${p(f.wallRight[i])}" stroke="#e4c674" stroke-width="1" opacity=".5"/>`);
        parts.push(`<circle cx="${mid[0]}" cy="${mid[1]}" r="2.3" fill="#c39dff"/>`);
      }
      parts.push('</g>');
      if((f.corridor||[]).length)parts.push(`<path id="corridorGuide" d="${path(f.corridor)}" stroke="#c39dff" stroke-width="2" stroke-dasharray="4 3" fill="none"/>`);
    }
    const showTrace=$("showTrace").checked||ui.step===1||ui.step===2;
    if(showTrace){
      parts.push(`<g id="searchEdges" opacity="${ui.step===2?.95:.7}">`);
      for(const e of f.trace){
        const color=e[6]===0?"#669de5":e[6]===4?"#758797":e[6]===2?"#ffb063":"#f77f91";
        if(ui.step===1&&e[6]!==0&&e[6]!==4)continue;
        parts.push(`<path data-result="${e[6]}" d="${path([[e[0],e[1]],[e[2],e[3]],[e[4],e[5]]])}" fill="none" stroke="${color}" stroke-width="${e[6]===4?.8:1.5}" opacity="${e[6]===4?.22:.75}"/>`);
      }
      parts.push(`</g>`);
    }
    const showPath=ui.step===-1||ui.step>=3;
    if(f.valid&&$("showEnvelope").checked&&(showPath||ui.step===2)){
      const half=(model.front+model.rear)/6,radius=Math.hypot(half,model.width/2)+model.margin;
      parts.push('<g id="bodyEnvelope" fill="#49d8c7" opacity=".06">');
      for(let i=0;i<f.path.length;i+=8){const state=f.path[i];for(let j=0;j<3;j++){
        const offset=-model.rear+half+2*half*j;
        const q=p([state[0]+offset*Math.cos(state[2]),state[1]+offset*Math.sin(state[2])]);
        parts.push(`<circle cx="${q[0]}" cy="${q[1]}" r="${radius*m.scale}"/>`);
      }}parts.push('</g>');
    }
    if(f.valid&&showPath){
      parts.push(`<path id="selectedPath" d="${path(f.path)}" fill="none" stroke="#49e4cf" stroke-width="3.5" stroke-linecap="round"/>`);
      for(let i=0;i<f.path.length;i+=5){const q=p(f.path[i]);parts.push(`<circle cx="${q[0]}" cy="${q[1]}" r="1.8" fill="#a0fff0"/>`);}
      const end=p(f.path[f.path.length-1]);parts.push(`<circle cx="${end[0]}" cy="${end[1]}" r="4" fill="#183e36" stroke="#67f0d9"/><text x="${end[0]+9}" y="${end[1]+4}" class="svg-small">현재 계획 끝</text>`);
    }
    if(f.reference&&(ui.step===-1||ui.step===4)){
      const r=f.reference,q=p(r),ahead=p([r[0]+.7*Math.cos(r[2]),r[1]+.7*Math.sin(r[2])]);
      parts.push(`<g id="referencePoint"><circle cx="${q[0]}" cy="${q[1]}" r="11" fill="#ffe185" opacity=".14"/><circle cx="${q[0]}" cy="${q[1]}" r="5" fill="#ffe185" stroke="#4e4326"/><path d="M${q}L${ahead}" stroke="#ffe185" stroke-width="2"/><text x="${q[0]+10}" y="${q[1]-15}" fill="#ffe185" font-size="11">약 1m 앞 목표</text></g>`);
    }
    const ego=p(f.ego),sc=m.scale;
    parts.push(`<g id="egoVehicle" transform="translate(${ego}) rotate(${-degrees(f.ego[2])})"><rect x="${-model.rear*sc}" y="${-model.width*sc/2}" width="${(model.front+model.rear)*sc}" height="${model.width*sc}" rx="${.08*sc}" fill="#bdd9f8" stroke="#f0f7ff" stroke-width="1.5"/><rect x="${model.front*.53*sc}" y="${-model.width*.34*sc}" width="${model.front*.17*sc}" height="${model.width*.68*sc}" rx="2" fill="#395574"/><path d="M${model.front*.84*sc},${-.1*sc}L${model.front*.99*sc},0L${model.front*.84*sc},${.1*sc}" fill="none" stroke="#2b4a60" stroke-width="1.5"/><circle r="3" fill="#14283d" stroke="#fff"/></g>`);
    if(!f.valid&&f.phase===4)parts.push(`<g id="holdNotice"><rect x="${ego[0]-38}" y="${ego[1]-51}" width="76" height="24" rx="5" fill="#492d2c" stroke="#d68f7b"/><text x="${ego[0]}" y="${ego[1]-35}" text-anchor="middle" fill="#ffd4bd" font-size="11" font-weight="700">HOLD · 경로 없음</text></g>`);
    parts.push('<text x="925" y="420" class="svg-small">+X →</text><text x="18" y="62" class="svg-small">+Y ↑</text>');
    $("map").innerHTML=parts.join("");
  }

  function drawChart() {
    const f=frame(), W=740,H=145, left=48,right=716,top=19,bottom=116;
    if(!f.valid){$("curvature").innerHTML=`<text x="370" y="75" text-anchor="middle" fill="#879db6" font-size="13">${f.phase===4?'HOLD · 출력 경로 없음':'zone 밖 · 기존 GPS 경로 사용'}</text>`;return;}
    const arc=[0];for(let i=1;i<f.path.length;i++)arc.push(arc[i-1]+Math.hypot(f.path[i][0]-f.path[i-1][0],f.path[i][1]-f.path[i-1][1]));
    const maxX=Math.max(2,Math.ceil(arc[arc.length-1])),limit=Math.tan(model.maxSteer||.476)/model.wheelbase;
    const x=s=>left+s/maxX*(right-left),y=k=>(top+bottom)/2-k/(limit*1.2)*(bottom-top)/2;
    let lines='';for(const v of [-limit,0,limit])lines+=`<path d="M${left},${y(v)}H${right}" stroke="${v===0?'#39516a':'#433844'}" stroke-dasharray="${v===0?'0':'4 4'}"/><text x="37" y="${y(v)+3}" text-anchor="end" fill="#7d94ae" font-size="9">${n(v,2)}</text>`;
    for(let i=0;i<=maxX;i+=2)lines+=`<text x="${x(i)}" y="134" text-anchor="middle" fill="#7d94ae" font-size="9">${i}m</text>`;
    lines+=`<path id="curvaturePath" d="${f.path.map((s,i)=>`${i?'L':'M'}${x(arc[i])},${y(Math.tan(s[4])/model.wheelbase)}`).join(' ')}" fill="none" stroke="#4be4d0" stroke-width="2"/>`;
    let ri=0;if(f.reference)for(let i=1;i<f.path.length;i++)if(Math.hypot(f.path[i][0]-f.reference[0],f.path[i][1]-f.reference[1])<Math.hypot(f.path[ri][0]-f.reference[0],f.path[ri][1]-f.reference[1]))ri=i;
    lines+=`<path d="M${x(arc[ri])},${top}V${bottom}" stroke="#e1c87d" stroke-dasharray="3 4"/><text x="${x(arc[ri])+6}" y="14" fill="#e1c87d" font-size="9">목표점</text><text x="7" y="12" fill="#7d94ae" font-size="9">κ [1/m]</text>`;
    $("curvature").innerHTML=lines;
  }

  function render() {
    const s=scene(), f=frame();
    $("sceneTitle").textContent=s.title;$("scenarioIndex").textContent=`SCENARIO ${String(ui.scenario+1).padStart(2,'0')} / ${data.scenarios.length}`;
    $("sceneDescription").textContent=s.description;
    $("testConditions").textContent=`1차로 ${n(s.laneWidth,1)}m · 최대 ${n(model.cruiseSpeed,1)}m/s · 유동 속도 · 차량 ${n(model.front+model.rear,2)} × ${n(model.width,2)}m · 10Hz`;
    $("phaseBadge").textContent=`${phaseNames[f.phase]} · ${phaseKo[f.phase]}`;
    $("phaseBadge").className=`state ${f.phase===4?'hold':f.complete?'complete':''}`;
    $("frameNote").textContent=`인지 ${f.perception?'ON':'OFF'} · GPS 부근 점유 ${f.detected?'있음':'없음'}`;
    $("timeline").max=s.frames.length-1;$("timeline").value=ui.frame;
    $("clock").textContent=`${n(f.time,1)} / ${n(s.frames[s.frames.length-1].time,1)}s`;
    $("frameLabel").textContent=`프레임 ${ui.frame+1} / ${s.frames.length}`;
    $("play").textContent=ui.playing?'일시정지':'재생';$("play").disabled=s.frames.length<2;
    $("first").disabled=$("previous").disabled=ui.frame===0;
    $("last").disabled=$("next").disabled=ui.frame===s.frames.length-1;
    $("pointCount").textContent=f.valid?`${f.path.length}개`:'0개';
    $("clearance").textContent=f.valid?`${n(f.clearance,3)}m`:'—';
    $("actualSpeed").textContent=`${n(f.ego[3],2)}m/s`;$("targetSpeed").textContent=f.valid?`${n(f.speedLimit,2)}m/s`:'HOLD';
    $("compute").textContent=`${n(f.ms,2)}ms`;$("stopDistance").textContent=f.stop>0?`${n(f.stop,2)}m`:'초기 검사 실패';
    if(f.reference){
      const dx=f.reference[0]-f.ego[0],dy=f.reference[1]-f.ego[1],a=f.ego[2];
      $("reference").innerHTML=`<div><span>x</span> ${n(Math.cos(a)*dx+Math.sin(a)*dy,3)}m</div><div><span>y</span> ${n(-Math.sin(a)*dx+Math.cos(a)*dy,3)}m</div><div><span>yaw</span> ${n(degrees(wrap(f.reference[2]-a)),1)}°</div><div><span>κ</span> ${n(Math.tan(f.reference[4])/model.wheelbase,3)}/m</div>`;
    }else $("reference").innerHTML=`<div style="grid-column:1/-1">${f.phase===4?'HOLD — 추종 목표점 없음':'기존 GPS 목표점 사용 · 회피 출력 없음'}</div>`;
    const explanation=ui.step<0?["벽 관측 → 통로 중간점 → 연결 → 차량 운동 검증", "zone 안에서 0.1초마다 연석과 장애물 사이의 자유 공간을 갱신합니다. GPS 진행 방향에 따라 이어지는 통로를 고르고 중앙을 연결합니다."]:stepText[ui.step];
    $("stepTitle").textContent=explanation[0];$("stepText").textContent=explanation[1];
    if(!f.valid)$("stepText").textContent=reasonText[f.reason]||f.reason;
    $("reason").textContent=f.complete?'후단의 출구 통과, 경로 정렬, 후속 경로를 독립 관측에서 확인했습니다.':reasonText[f.reason]||f.reason;
    $("outcome").textContent=`기록 결과: ${s.outcome==='complete'?'출구 통과':s.outcome==='hold'?(s.expectedHold?'예상 HOLD':'완주 실패 · HOLD'):s.outcome==='unsafe'?'접촉 검출':'기록 종료'} · 검사상 차량 접촉 ${f.vehicleContact?'있음':'없음'} · 연석 접촉 ${f.curbContact?'있음':'없음'}`;
    $("counts").innerHTML=[['검증 구간',f.counts[0]],['차체·동역학 탈락',f.counts[1]],['제동 탈락',f.counts[2]],['방향 탈락',f.counts[3]],['중복 제거',f.counts[4]]].map(([label,count],i)=>`<span class="${i===1||i===2?'bad':''}">${label} ${count}</span>`).join('');
    document.querySelectorAll('.scenario').forEach((b,i)=>{b.hidden=ui.count!==0&&data.scenarios[i].boxes.length!==ui.count;b.classList.toggle('selected',i===ui.scenario);b.setAttribute('aria-pressed',String(i===ui.scenario));});
    for(const [id,count] of [['cases2',2],['cases3',3],['casesAll',0]]){$(id).classList.toggle('selected',ui.count===count);}
    document.querySelectorAll('[data-step]').forEach(b=>{const active=Number(b.dataset.step)===ui.step;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
    for(const id of ['overview','focus']){$(id).classList.toggle('selected',(id==='focus')===ui.focus);$(id).setAttribute('aria-pressed',String((id==='focus')===ui.focus));}
    $("allSteps").classList.toggle('selected',ui.step<0);
    $("tooltip").hidden=true;drawMap();drawChart();
  }
  function setFrame(i) {ui.frame=Math.max(0,Math.min(scene().frames.length-1,Math.round(i)));render();}
  function setScenario(i) {
    ui.scenario=Math.max(0,Math.min(data.scenarios.length-1,i));ui.frame=0;ui.playing=false;ui.accumulator=0;
    const s=scene();const markers=[];
    if(ui.count&&ui.count!==s.boxes.length)ui.count=s.boxes.length;
    s.boxes.forEach((b,i)=>{let best=0;for(let j=1;j<s.frames.length;j++)if(Math.abs(s.frames[j].ego[0]-(b[0]-1))<Math.abs(s.frames[best].ego[0]-(b[0]-1)))best=j;markers.push([`차량 ${i+1} 앞`,best]);});
    markers.push([s.outcome==='complete'?'출구 확인':'HOLD 확인',s.frames.length-1]);
    $("milestones").innerHTML=markers.map(([label,index])=>`<button data-frame="${index}">${esc(label)}</button>`).join('');
    $("milestones").querySelectorAll('button').forEach(b=>b.onclick=()=>{ui.playing=false;setFrame(Number(b.dataset.frame));});render();
  }
  function setStep(step) {
    ui.step=step;
    $("showTrace").checked=step===1||step===2;$("showEnvelope").checked=step!==0&&step!==1;
    if(step===1||step===2||step===4)ui.focus=true;
    render();
  }
  function togglePlay() {
    if(scene().frames.length<2)return;
    if(ui.frame===scene().frames.length-1)ui.frame=0;
    ui.playing=!ui.playing;ui.accumulator=0;render();
  }
  $("scenarios").innerHTML=data.scenarios.map((s,i)=>`<button class="scenario" data-index="${i}"><span class="scenario-number">${String(i+1).padStart(2,'0')}</span><span class="scenario-name">${esc(s.title)}</span><span class="result-chip ${s.outcome}">${s.outcome==='complete'?'완주 기록':s.outcome==='hold'?(s.expectedHold?'예상 HOLD':'완주 실패'):s.outcome==='unsafe'?'접촉 검출':'기록 종료'}</span></button>`).join('');
  document.querySelectorAll('.scenario').forEach(b=>b.onclick=()=>setScenario(Number(b.dataset.index)));
  for(const [id,count] of [['cases2',2],['cases3',3],['casesAll',0]])$(id).onclick=()=>{
    ui.count=count;if(count&&scene().boxes.length!==count)setScenario(data.scenarios.findIndex(s=>s.boxes.length===count));else render();
  };
  document.querySelectorAll('[data-step]').forEach(b=>b.onclick=()=>setStep(Number(b.dataset.step)));
  $("allSteps").onclick=()=>setStep(-1);$("overview").onclick=()=>{ui.focus=false;render();};$("focus").onclick=()=>{ui.focus=true;render();};
  $("first").onclick=()=>{ui.playing=false;setFrame(0);};$("last").onclick=()=>{ui.playing=false;setFrame(scene().frames.length-1);};
  $("previous").onclick=()=>{ui.playing=false;setFrame(ui.frame-1);};$("next").onclick=()=>{ui.playing=false;setFrame(ui.frame+1);};
  $("play").onclick=togglePlay;$("timeline").oninput=e=>{ui.playing=false;setFrame(Number(e.target.value));};
  for(const id of ['showGrid','showEnvelope','showTrace','showHistory'])$(id).onchange=()=>{ui.step=-1;render();};
  document.addEventListener('keydown',e=>{
    if(/INPUT|SELECT|BUTTON|TEXTAREA/.test(e.target.tagName))return;
    if(e.code==='Space'){e.preventDefault();togglePlay();}
    if(e.code==='ArrowRight'||e.code==='ArrowLeft'){e.preventDefault();ui.playing=false;setFrame(ui.frame+(e.code==='ArrowRight'?1:-1));}
  });
  $("map").addEventListener('mousemove',e=>{
    const f=frame();if(!f.valid||!transform)return;
    const pos=$("map").createSVGPoint();pos.x=e.clientX;pos.y=e.clientY;
    const at=pos.matrixTransform($("map").getScreenCTM().inverse());
    let index=-1,best=17;f.path.forEach((s,i)=>{const q=transform.point(s),d=Math.hypot(q[0]-at.x,q[1]-at.y);if(d<best){best=d;index=i;}});
    if(index<0){$("tooltip").hidden=true;return;}
    const s=f.path[index];const wrapRect=$("map").parentElement.getBoundingClientRect();
    $("tooltip").innerHTML=`<b>경로 점 ${index+1}/${f.path.length}</b><br>고정 좌표 x ${n(s[0],3)}, y ${n(s[1],3)}m<br>yaw ${n(degrees(s[2]),1)}° · κ ${n(Math.tan(s[4])/model.wheelbase,3)}/m`;
    $("tooltip").style.left=`${Math.max(6,Math.min(wrapRect.width-240,e.clientX-wrapRect.left+14))}px`;
    $("tooltip").style.top=`${Math.max(6,Math.min(wrapRect.height-90,e.clientY-wrapRect.top-88))}px`;$("tooltip").hidden=false;
  });
  $("map").addEventListener('mouseleave',()=>$("tooltip").hidden=true);
  function animate(time) {
    if(ui.playing){
      ui.accumulator+=Math.min(time-ui.lastTime,300)*Number($("speed").value);
      const steps=Math.floor(ui.accumulator/(data.stepSeconds*1000));
      if(steps){ui.accumulator-=steps*data.stepSeconds*1000;ui.frame=Math.min(scene().frames.length-1,ui.frame+steps);if(ui.frame===scene().frames.length-1)ui.playing=false;render();}
    }
    ui.lastTime=time;requestAnimationFrame(animate);
  }
  $("provenance").textContent=`생성 ${data.generated||''} · core ${data.coreHash?.slice(0,12)||''} · 조건 ${data.fixtureHash?.slice(0,12)||''} · 총 ${data.scenarios.reduce((a,s)=>a+s.frames.length,0)} 프레임`;
  setScenario(Math.max(0,data.scenarios.findIndex(s=>s.id==='s_lr_2')));requestAnimationFrame(animate);
  window.V2Lab={data,ui,scene,frame,setScenario,setFrame,setStep,togglePlay,render,ready:true};
})();
