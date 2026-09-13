"""Retain COCO IDs and all 80 names, using only the 2726 approved/retained images."""
import json
from pathlib import Path
import shutil
import yaml
from ultralytics import YOLO

root=Path(__file__).resolve().parents[2]
source=root/'datasets/traffic_training_4000/new_2726'
output=root/'datasets/traffic_incremental_80/dataset'
if output.exists():raise SystemExit('Output exists; refusing to overwrite')
base=YOLO(str(root/'src/stack_traffic/models/yolov8n.pt'))
assert len(base.names)==80 and base.names[9]=='traffic light'
records=json.loads((source/'manifest.json').read_text())['images']
assert len(records)==2726
for split in ('train','val'):
 for kind in ('images','labels'):(output/kind/split).mkdir(parents=True)
for record in records:
 split='val' if record['session']=='150256' else 'train'
 name=record['image']
 shutil.copy2(source/'images'/name,output/'images'/split/name)
 old=(source/'labels'/(Path(name).stem+'.txt')).read_text().splitlines()
 rows=[]
 for line in old:
  parts=line.split();assert len(parts)==5 and parts[0]=='0'
  rows.append('9 '+' '.join(parts[1:])+'\n')
 (output/'labels'/split/(Path(name).stem+'.txt')).write_text(''.join(rows))
 record['split']=split
(output/'data.yaml').write_text(yaml.safe_dump(dict(path=str(output),train='images/train',val='images/val',names=base.names),sort_keys=False))
(output/'manifest.json').write_text(json.dumps(dict(images=records,classes=base.names,class_id=9),ensure_ascii=False,indent=2))
summary=dict(total=2726,train=sum(r['split']=='train' for r in records),val=sum(r['split']=='val' for r in records),num_classes=80,traffic_light_id=9)
(output/'summary.json').write_text(json.dumps(summary,indent=2));print(summary)
