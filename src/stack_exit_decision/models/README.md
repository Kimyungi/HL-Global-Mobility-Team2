# Exit decision model

`exit_decision_yolo26n.pt` detects the three-panel exit signal as one bounding box.
The original model-only PR #83 is now connected to MGM through `stack_exit_decision.node`.

## Classes

| ID | Name | Signal pattern (left to right) | Mission direction |
|---:|---|---|---|
| 0 | `red_blue_red` | red-blue-red | `Right` |
| 1 | `blue_red_red` | blue-red-red | `Left` |

The mission direction mapping was confirmed by the user on 2026-09-17 for
`Last_mission_state`. The detector emits the signal class; application logic
must translate that class into the mission direction.

Current Yongin policy: entering route 05 zone [2] (indices 84–103) starts
exit inference while navigation continues. Reaching CSV state=3 (index 92)
requests a stop. After the vehicle stops, MGM observes for three seconds and
selects route 06 for Left or route 07 for Right; no decision or a tie selects 06.
Approach detections are not included in the stationary vote window.
See [the mission contract](../../../docs/LAST_MISSION_STATE.md).

## Training summary

- Base architecture: Ultralytics YOLO26n
- Input size: 640
- Positive images: 90 per class
- Background images: 60
- Validation-selected checkpoint: epoch 37
- Held-out test: precision 0.944, recall 0.998, mAP50 0.962, mAP50-95 0.808
- SHA-256: `f99499b7e6e1d13f3f1480ac20026c2a8c3aded7ab2b54d53372ecbda0873e12`

The training and held-out test images originate from the same collection session. Validate on a separately captured drive before using the model for vehicle decisions.
