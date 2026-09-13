"""Start fine-tuning the existing YOLOv8n on the audited 4000-image dataset.

Training writes a candidate checkpoint only. A running vehicle's active weights
are not overwritten by this script.
"""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--epochs',type=int,default=30)
    parser.add_argument('--device',default='cpu')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    dataset=root/'datasets/traffic_training_4000/dataset'
    output=root/'datasets/traffic_training_4000/runs'
    from ultralytics import YOLO
    model=YOLO(str(root/'src/stack_traffic/models/yolov8n.pt'))
    model.train(data=str(dataset/'data.yaml'),epochs=args.epochs,imgsz=512,
                batch=8,device=args.device,workers=2,seed=20260908,
                project=str(output),name='yolov8n_center_4000',exist_ok=False,
                pretrained=True,patience=8,freeze=10,cache=False,
                mosaic=0.0,mixup=0.0,fliplr=0.0,translate=0.05,
                scale=0.2,plots=False,save=True)
    best=Path(model.trainer.best)
    evaluation=YOLO(str(best)).val(data=str(dataset/'data.yaml'),
        split='test',device=args.device,imgsz=512,batch=8,plots=False)
    report=dict(checkpoint=str(best),test_metrics=evaluation.results_dict,
                model_names=YOLO(str(best)).names,applied_to_ros=False)
    (output/'training_result.json').write_text(json.dumps(report,indent=2))
    print('Training and held-out test evaluation complete:',best)


if __name__=='__main__':
    main()
