"""Learn traffic-light scores from 2726 images while locking other COCO outputs."""
import hashlib
import json
from pathlib import Path
import torch
from ultralytics import YOLO
from preserve_traffic_head import PreserveTrafficTrainer, TrafficRowGuard

ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/'datasets/traffic_incremental_80'
DATA=WORK/'dataset/data.yaml'
WEIGHTS=ROOT/'src/stack_traffic/models/yolov8n.pt'
SOURCE_HASH=hashlib.sha256(WEIGHTS.read_bytes()).hexdigest()


def main():
    # Preflight the dependency that previously prevented checkpoint saving.
    import polars
    assert polars.__version__
    baseline=YOLO(str(WEIGHTS))
    assert len(baseline.names)==80 and baseline.names[9]=='traffic light'
    metrics=baseline.val(data=str(DATA),imgsz=512,batch=16,device='cpu',
                         plots=False,project=str(WORK/'runs'),name='baseline',workers=0)
    before={k:float(v) for k,v in metrics.results_dict.items()}
    (WORK/'baseline_metrics.json').write_text(json.dumps(before,indent=2))
    model=YOLO(str(WEIGHTS))
    model.train(trainer=PreserveTrafficTrainer,data=str(DATA),
        epochs=30,patience=8,imgsz=512,batch=16,device='cpu',workers=0,
        optimizer='AdamW',lr0=.001,weight_decay=0.0,
        freeze=22,single_cls=False,seed=20260908,amp=False,
        mosaic=0.0,mixup=0.0,fliplr=0.0,translate=0.0,scale=0.0,
        hsv_h=0.0,hsv_s=0.0,hsv_v=0.0,
        project=str(WORK/'runs'),name='yolov8n_80class_incremental',
        plots=False,save=True,cache=False)
    best=Path(model.trainer.best)
    candidate=YOLO(str(best)).model.eval()
    reference=YOLO(str(WEIGHTS)).model.eval()
    assert candidate.names==reference.names
    guard=TrafficRowGuard(reference)
    guard.assert_preserved(candidate)
    torch.manual_seed(20260908)
    image=torch.rand(1,3,128,128)
    with torch.no_grad():
        expected=reference(image)[0]
        actual=candidate(image)[0]
    channels=[i for i in range(84) if i!=13]
    assert torch.equal(expected[:,channels],actual[:,channels])
    measured=YOLO(str(best)).val(data=str(DATA),imgsz=512,batch=16,
        device='cpu',plots=False,project=str(WORK/'runs'),name='candidate',workers=0)
    after={k:float(v) for k,v in measured.results_dict.items()}
    assert SOURCE_HASH==hashlib.sha256(WEIGHTS.read_bytes()).hexdigest()
    passed=all(after[k]>=before[k] for k in ('metrics/mAP50-95(B)','metrics/recall(B)'))
    report=dict(checkpoint=str(best),baseline=before,candidate=after,
        frozen_weights_preserved=True,raw_other_class_outputs_identical=True,
        traffic_validation_passed=passed,source_sha256=SOURCE_HASH,
        applied_to_ros=False)
    (WORK/'training_result.json').write_text(json.dumps(report,indent=2))
    print('Training finished. Preservation verified; traffic validation passed:',passed,flush=True)


if __name__=='__main__':
    main()
