"""OAK-D Pro에서 캘리브레이션용 정지 프레임 한 장을 저장한다 — depthai v3 API판.

capture_calibration_frame.py(v2 API)와 동일한 용도·조작이지만 depthai 3.x가
설치된 PC용이다 (v2 API인 createColorCamera가 3.x에 없어 원본 스크립트가
실행되지 않음 — 2026-08-11 산업용 PC에서 확인). 해상도는 stack_lane 노드와
동일한 1280x720 / CAM_A 를 사용하므로 캘리브레이션 결과가 노드에 그대로 통용된다.

카메라를 최종 장착 위치/각도 그대로 켜둔 상태에서, 바닥에 기준점 마커를 다
배치한 뒤 실행할 것. 나중에 브래킷을 다시 조이면 재촬영·재캘리브레이션 필요.
저장한 사진은 calibrate_homography.py(depthai 무관, cv2만 사용)로 이어서 처리.

화면이 검게 나올 때의 진단 순서 (2026-08-11 현장 이슈):
  1. 터미널에 1초마다 찍히는 "밝기" 값을 본다.
     - 밝기 < 15  = 카메라에 실제로 어두운 장면이 들어오는 것 (야간 등).
       → 마커를 전조등/작업등으로 비추고, 필요시 --ae-comp 3 (자동노출 보정 +)
         또는 --exposure-us 20000 --iso 800 (수동 노출)로 재실행.
     - 밝기 정상(50+)인데 창만 검다 = 디스플레이 문제 → --no-gui 로 저장하고
       사진 파일을 열어 확인.
  2. "이미 사용 중" 오류가 나면 이전 실행(창)이 카메라를 물고 있는 것 — 먼저 종료.

사용:
  python3 capture_calibration_frame_v3.py                      # 창 보고 SPACE로 저장
  python3 capture_calibration_frame_v3.py --no-gui             # 워밍업 후 자동 저장
  python3 capture_calibration_frame_v3.py --ae-comp 3          # 어두울 때 노출 올림
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import depthai as dai

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "captures" / "calib_frame.png"
# 차선용 OAK-D Pro 실측 MxID (2026-08-11 확정 — stack_lane/node.py camera_mxid와 동일값)
DEFAULT_MXID = "14442C105157D3D200"
WINDOW = "calibration capture — SPACE=save, q=quit"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="저장할 이미지 경로")
    parser.add_argument("--warmup", type=int, default=30, help="노출 적응 위해 먼저 버릴 프레임 수")
    parser.add_argument("--mxid", default=DEFAULT_MXID, help="OAK-D MxID (빈 문자열 = 첫 가용 장치)")
    parser.add_argument("--no-gui", action="store_true", help="창 없이 워밍업 직후 자동 저장")
    parser.add_argument("--ae-comp", type=int, default=0,
                        help="자동노출 보정 -9~9 (어두우면 양수, 예: 3)")
    parser.add_argument("--exposure-us", type=int, default=0,
                        help="수동 노출 시간[us] (0=자동). --iso와 함께 지정")
    parser.add_argument("--iso", type=int, default=800, help="수동 노출 시 ISO (--exposure-us와 함께)")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    device = dai.Device(dai.DeviceInfo(args.mxid)) if args.mxid else dai.Device()
    pipeline = dai.Pipeline(device)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    if args.exposure_us > 0:
        cam.initialControl.setManualExposure(args.exposure_us, args.iso)
        print(f"수동 노출: {args.exposure_us}us / ISO {args.iso}")
    elif args.ae_comp != 0:
        cam.initialControl.setAutoExposureCompensation(args.ae_comp)
        print(f"자동노출 보정: {args.ae_comp:+d}")
    q = cam.requestOutput((1280, 720), dai.ImgFrame.Type.BGR888i).createOutputQueue(maxSize=4, blocking=False)
    pipeline.start()

    if not args.no_gui:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)
        print("카메라 부팅/노출 적응 중... 마커가 다 보이면 SPACE로 저장, q로 취소")

    i = 0
    while True:
        pkt = q.get()
        frame = pkt.getCvFrame()
        i += 1
        if i % 30 == 0:
            print(f"  frame {i}, 밝기(평균 0~255): {frame.mean():.0f}")
        if i < args.warmup:
            continue
        if args.no_gui:
            cv2.imwrite(str(out_path), frame)
            print(f"저장됨: {out_path} (밝기 {frame.mean():.0f})")
            break
        cv2.imshow(WINDOW, frame)
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
