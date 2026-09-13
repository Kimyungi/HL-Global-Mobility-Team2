"""Build exactly 4000 images, holding out a complete Yongin session."""
import hashlib
import json
from pathlib import Path
import random
import shutil

ROOT=Path('datasets/traffic_training_4000').resolve()
OUT=ROOT/'dataset'
if OUT.exists():raise SystemExit('Dataset output already exists; refusing overwrite')
new=json.loads((ROOT/'new_2726/manifest.json').read_text())['images']
old=json.loads((ROOT/'coco_1274/manifest.json').read_text())['images']
assert len(new)==2726 and len(old)==1274
assert len({r['image_sha256'] for r in new+old})==4000
# Keep the complete 15:02:56 capture out of training; adjacent video frames
# must not be randomly distributed between train and validation.
old_ids=[r['image'] for r in old]
random.Random(20260908).shuffle(old_ids)
old_split={name:('test' if i<127 else 'val' if i<254 else 'train') for i,name in enumerate(old_ids)}
for part in ('train','val','test'):
 for kind in ('images','labels'):(OUT/kind/part).mkdir(parents=True)
exported=[]
for group,rows in [('new_2726',new),('coco_1274',old)]:
 for row in rows:
  split=('val' if row['session']=='150256' else 'train') if group=='new_2726' else old_split[row['image']]
  name=row['image'];label=Path(name).stem+'.txt'
  shutil.copy2(ROOT/group/'images'/name,OUT/'images'/split/name)
  shutil.copy2(ROOT/group/'labels'/label,OUT/'labels'/split/label)
  exported.append({**row,'source_group':group,'split':split})
(OUT/'data.yaml').write_text(f'path: {OUT}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: traffic light\n')
(OUT/'manifest.json').write_text(json.dumps({'images':exported,'seed':20260908,'validation_yongin_session':'150256'},ensure_ascii=False,indent=2))
summary={p:sum(r['split']==p for r in exported) for p in ('train','val','test')}
summary.update(total=4000,new=2726,coco=1274)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
print(summary)
