# Exit decision model

`exit_decision_yolo26n.pt` detects the three-panel exit signal as one bounding box.
Runtime integration is intentionally excluded from this model-only change.

## Classes

| ID | Name | Signal pattern |
|---:|---|---|
| 0 | `red_blue_red` | red-blue-red |
| 1 | `blue_red_red` | blue-red-red |

## Training summary

- Base architecture: Ultralytics YOLO26n
- Input size: 640
- Positive images: 90 per class
- Background images: 60
- Validation-selected checkpoint: epoch 37
- Held-out test: precision 0.944, recall 0.998, mAP50 0.962, mAP50-95 0.808
- SHA-256: `f99499b7e6e1d13f3f1480ac20026c2a8c3aded7ab2b54d53372ecbda0873e12`

The training and held-out test images originate from the same collection session. Validate on a separately captured drive before using the model for vehicle decisions.
