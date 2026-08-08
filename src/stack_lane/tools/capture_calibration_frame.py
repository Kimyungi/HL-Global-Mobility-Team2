"""OAK-D Pro에서 캘리브레이션용 정지 프레임 한 장을 저장한다.

캘리브레이션은 이 스크립트로 찍은 사진 한 장 위에서 마커 픽셀 좌표를 클릭해
진행한다 (calibrate_homography.py). 카메라를 최종 장착 위치/각도(140cm 높이,
pitch 적용) 그대로 켜둔 상태에서, 바닥에 기준점 마커를 다 배치한 뒤 실행할 것.
나중에 브래킷을 다시 조이면 재촬영·재캘리브레이션 필요.

사용:
  python3 capture_calibration_frame.py --out ../captures/calib_frame.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from visualize_yolopv2 import oak_frames  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "captures" / "calib_frame.png"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="저장할 이미지 경로")
    parser.add_argument("--warmup", type=int, default=30, help="노출 적응 위해 먼저 버릴 프레임 수")
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print("카메라 부팅/노출 적응 중... 마커가 다 보이면 SPACE로 저장, q로 취소")
    for i, frame in enumerate(oak_frames()):
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
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
