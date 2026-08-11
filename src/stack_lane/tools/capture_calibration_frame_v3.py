"""OAK-D Pro에서 캘리브레이션용 정지 프레임 한 장을 저장한다 — depthai v3 API판.

capture_calibration_frame.py(v2 API)와 동일한 용도·조작이지만 depthai 3.x가
설치된 PC용이다 (v2 API인 createColorCamera가 3.x에 없어 원본 스크립트가
실행되지 않음 — 2026-08-11 산업용 PC에서 확인). 해상도는 stack_lane 노드와
동일한 1280x720 / CAM_A 를 사용하므로 캘리브레이션 결과가 노드에 그대로 통용된다.

카메라를 최종 장착 위치/각도 그대로 켜둔 상태에서, 바닥에 기준점 마커를 다
배치한 뒤 실행할 것. 나중에 브래킷을 다시 조이면 재촬영·재캘리브레이션 필요.
저장한 사진은 calibrate_homography.py(depthai 무관, cv2만 사용)로 이어서 처리.

사용:
  python3 capture_calibration_frame_v3.py --out ../captures/calib_frame.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import depthai as dai

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "captures" / "calib_frame.png"
# 차선용 OAK-D Pro 실측 MxID (2026-08-11 확정 — stack_lane/node.py camera_mxid와 동일값)
DEFAULT_MXID = "14442C105157D3D200"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="저장할 이미지 경로")
    parser.add_argument("--warmup", type=int, default=30, help="노출 적응 위해 먼저 버릴 프레임 수")
    parser.add_argument("--mxid", default=DEFAULT_MXID, help="OAK-D MxID (빈 문자열 = 첫 가용 장치)")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    device = dai.Device(dai.DeviceInfo(args.mxid)) if args.mxid else dai.Device()
    pipeline = dai.Pipeline(device)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q = cam.requestOutput((1280, 720), dai.ImgFrame.Type.BGR888i).createOutputQueue(maxSize=4, blocking=False)
    pipeline.start()

    print("카메라 부팅/노출 적응 중... 마커가 다 보이면 SPACE로 저장, q로 취소")
    i = 0
    while True:
        pkt = q.get()
        frame = pkt.getCvFrame()
        i += 1
        if i < args.warmup:
            continue
        cv2.imshow("calibration capture — SPACE=save, q=quit", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            cv2.imwrite(str(out_path), frame)
            print(f"저장됨: {out_path}")
            break
        if key == ord("q"):
            print("취소")
            break
    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
