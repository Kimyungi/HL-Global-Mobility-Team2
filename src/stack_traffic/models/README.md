# YOLO 모델 위치

배포할 신호등 검출 모델을 이 폴더에 `yolov8n.pt`라는 이름으로 배치한 뒤
`stack_traffic` 패키지를 다시 빌드한다.
모델 파일은 `.gitignore`로 제외되므로 이 저장소에는 포함되지 않는다.

다른 위치의 모델을 사용하려면 실행 시 다음 파라미터로 지정한다.

```bash
-p model_path:=/absolute/path/to/model.pt
```
