# YOLO 모델 위치

배포할 신호등 검출 모델을 이 폴더에 `yolov8n.pt`라는 이름으로 배치한 뒤
`stack_traffic` 패키지를 다시 빌드한다.
기본 `yolov8n.pt`는 패키지와 함께 배포되므로 별도 경로 설정 없이 사용할 수 있다.

다른 위치의 모델을 사용하려면 실행 시 다음 파라미터로 지정한다.

```bash
-p model_path:=/absolute/path/to/model.pt
```
