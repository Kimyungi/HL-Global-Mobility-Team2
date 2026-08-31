# 신호등 모델

## 객체 검출

`yolov8n.pt`가 신호등 bbox를 검출한다. 다른 YOLO 모델을 쓰려면 실행 시
다음 파라미터로 지정한다.

```bash
-p model_path:=/absolute/path/to/model.pt
```

신호등 색상은 검출된 bbox 안에서 HSV 비율로만 판정한다. 별도 ONNX 색상
분류기는 실험 결과 대비 연산·설정 복잡도가 커서 실행 경로와 설치 대상에서
제외했다.

정지선은 YOLO 모델로 검출하지 않는다. 주간 흰색 띠, 야간 국소 대비, 수평
평행 에지 쌍을 병렬로 검사한 뒤 공통 형상·안정성 조건을 적용한다. 실차 실행과
확인 명령은 [`RUNBOOK_full_operation_20260830.md`](../../adas_mgm/RUNBOOK_full_operation_20260830.md)를 따른다.
