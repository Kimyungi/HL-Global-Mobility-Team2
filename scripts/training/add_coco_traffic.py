"""Reproducible COCO traffic-light subset with the same central crop."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import random
import shutil
import time
from zipfile import ZipFile
import requests
from PIL import Image

ROOT=Path('datasets/traffic_training_4000').resolve()
OUT=ROOT/'coco_1274'
OUT.mkdir(exist_ok=True)
for d in ('images','labels','records'):(OUT/d).mkdir(exist_ok=True)
SEED=20260908
candidates=[]
with ZipFile(ROOT/'coco_source/coco2017labels.zip') as archive:
    for name in sorted(archive.namelist()):
        if '/labels/train2017/' not in name or not name.endswith('.txt'):continue
        boxes=[list(map(float,line.split()[1:])) for line in archive.read(name).decode().splitlines() if line.split()[0]=='9']
        if any(min(x+w/2,.7)>max(x-w/2,.3) and min(y+h/2,.5)>max(y-h/2,0) for x,y,w,h in boxes):
            candidates.append((Path(name).stem,boxes))
selected=random.Random(SEED).sample(candidates,1274)
(ROOT/'coco_selection.json').write_text(json.dumps({'seed':SEED,'candidate_count':len(candidates),'selected_ids':[r[0] for r in selected]},indent=2))

def prepare(item):
    image_id,boxes=item
    record_path=OUT/'records'/(image_id+'.json')
    if record_path.exists():return json.loads(record_path.read_text())
    url=f'https://s3.amazonaws.com/images.cocodataset.org/train2017/{image_id}.jpg'
    for attempt in range(5):
        try:
            r=requests.get(url,timeout=(15,45))
            r.raise_for_status()
            data=r.content
            break
        except requests.RequestException:
            if attempt==4:raise RuntimeError('COCO download failed: '+image_id) from None
            time.sleep(2**attempt)
    with Image.open(io.BytesIO(data)) as image:
        image=image.convert('RGB');width,height=image.size
        bounds=(round(width*.3),0,round(width*.7),round(height*.5))
        cropped=image.crop(bounds)
        cw,ch=cropped.size
        rows=[]
        for x,y,w,h in boxes:
            x1=max(bounds[0],(x-w/2)*width);x2=min(bounds[2],(x+w/2)*width)
            y1=max(bounds[1],(y-h/2)*height);y2=min(bounds[3],(y+h/2)*height)
            if x2<=x1 or y2<=y1:continue
            rows.append(f'0 {((x1+x2)/2-bounds[0])/cw:.8f} {((y1+y2)/2-bounds[1])/ch:.8f} {(x2-x1)/cw:.8f} {(y2-y1)/ch:.8f}\n')
        if not rows:raise RuntimeError('No box after integer crop: '+image_id)
        name='coco_'+image_id+'.jpg'
        cropped.save(OUT/'images'/name,quality=95)
    (OUT/'labels'/('coco_'+image_id+'.txt')).write_text(''.join(rows))
    result=dict(image=name,coco_id=image_id,source_url=url,width=cw,height=ch,crop_xyxy=bounds,
                boxes=len(rows),source_image_sha256=hashlib.sha256(data).hexdigest(),
                image_sha256=hashlib.sha256((OUT/'images'/name).read_bytes()).hexdigest(),
                annotation_status='coco_ground_truth',source_class=9,class_id=0)
    record_path.write_text(json.dumps(result,indent=2))
    return result

with ThreadPoolExecutor(max_workers=10) as pool:
    rows=[]
    for row in pool.map(prepare,selected):
        rows.append(row)
        if len(rows)%100==0:print('COCO prepared',len(rows),'/1274',flush=True)
assert len({r['image_sha256'] for r in rows})==1274
(OUT/'manifest.json').write_text(json.dumps({'classes':['traffic light'],'seed':SEED,'images':rows},indent=2))
print('COCO subset ready: 1274',flush=True)
