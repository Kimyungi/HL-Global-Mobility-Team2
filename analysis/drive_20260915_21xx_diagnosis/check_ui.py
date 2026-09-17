from pathlib import Path
import subprocess
out=Path(__file__).resolve().parent
html=(out/'index.html').read_text()
harness='''<script>setTimeout(()=>{let count=0;const ok=(v,m)=>{if(!v)throw Error(m);count++};try{ok(DATA.length===2,'sessions');for(let n=0;n<2;n++){document.getElementById('run').value=n;document.getElementById('run').onchange();ok(frames.length>100,'frames');ok(document.getElementById('badge').textContent.length>0,'badge');ok(document.getElementById('events').children.length>0,'events');document.getElementById('slider').value=frames.length-1;document.getElementById('slider').oninput();ok(n===0?!frames[idx].manual:frames[idx].manual,'manual endpoint');document.getElementById('manual').checked=false;document.getElementById('manual').onchange();ok(frames.every(f=>!f.manual),'manual hidden');document.getElementById('manual').checked=true;document.getElementById('manual').onchange();document.getElementById('events').firstChild.click();ok(idx>=0&&idx<frames.length,'jump');}document.body.insertAdjacentHTML('beforeend','<pre id="ui-check">PASS '+count+'</pre>')}catch(e){document.body.insertAdjacentHTML('beforeend','<pre id="ui-check">FAIL '+e.stack+'</pre>')}},250)</script>'''
p=out/'ui_check.html';p.write_text(html.replace('</html>',harness+'</html>'))
chrome=Path.home()/'.cache/ms-playwright/chromium-1234/chrome-linux64/chrome'
for width in [1440,390]:
 r=subprocess.run([str(chrome),'--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage',f'--window-size={width},1000','--virtual-time-budget=1500','--dump-dom',p.as_uri()],capture_output=True,text=True,timeout=30)
 import re
 found=re.findall(r'<pre id="ui-check">(.*?)</pre>',r.stdout,re.S)
 result=found[-1] if found else 'NO RESULT '+r.stderr[-400:]
 print(width,result)
 assert found and re.fullmatch(r'PASS [0-9]+',result),result
p.unlink()
