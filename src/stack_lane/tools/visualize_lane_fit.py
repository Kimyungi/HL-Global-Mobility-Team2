"""전체 차선 경로 추정 파이프라인 라이브 확인 도구.

YOLOPv2 -> BEV -> 슬라이딩 윈도우 -> 중심선 -> lookahead 지점(x,y,yaw,curvature)
+ confidence까지 전 과정을 라이브로 오버레이해서 눈으로 확인한다.

호모그래피가 없으면(config/homography.json 미존재) 카메라 스펙 기반 placeholder를
자동 사용한다 — 실측 캘리브레이션 전까지 알고리즘 검증용 (PROJECT_BRIEF.md §9).
실측 후엔 같은 경로에 파일만 놓으면 코드 변경 없이 자동으로 전환된다.

시각화 로직은 stack_lane.debug_draw로 옮겨서 node.py(실차 디버그 이미지 발행)와
공유한다 — 이 스크립트는 프레임 소스 + 창 표시/저장만 담당.

사용:
  python3 visualize_lane_fit.py --source oak --device 0
  python3 visualize_lane_fit.py --source <이미지/영상> --device 0 --headless --save out.mp4
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from visualize_yolopv2 import frame_source  # noqa: E402

from stack_lane.bev import BevGrid, load_homography  # noqa: E402
from stack_lane.debug_draw import build_debug_frame  # noqa: E402
from stack_lane.lane_path import estimate_lane_path  # noqa: E402
from stack_lane.logging_utils import CsvFrameLogger  # noqa: E402
from stack_lane.yolopv2_infer import (  # noqa: E402
    DEFAULT_WEIGHTS,
    infer,
    load_model,
    preprocess,
    resolve_device,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    parser.add_argument("--source", default="oak")
    parser.add_argument("--img-size", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--lookahead", type=float, default=3.0)
    parser.add_argument("--homography", default=None, help="config/homography.json 경로 (없으면 자동 탐색 후 placeholder)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--save", default=None)
    parser.add_argument("--log-csv", default=None, help="프레임별 진단 데이터를 저장할 CSV 경로 (디버깅용)")
    args = parser.parse_args()

    device, half = resolve_device(args.device)
    model = load_model(args.weights, device, half)

    H, is_placeholder, meta = load_homography(args.homography)
    if is_placeholder:
        print("[경고] 실측 호모그래피 없음 — analytic placeholder 사용 중 "
              "(알고리즘 검증용, 실좌표 정확도 보장 안 함)")
        print(f"       placeholder 파라미터: {meta.get('placeholder_params')}")
    grid = BevGrid()
    H_inv = np.linalg.inv(H)

    logger = CsvFrameLogger(args.log_csv, is_placeholder_homography=is_placeholder) if args.log_csv else None
    if logger:
        print(f"CSV 로깅: {args.log_csv}")

    writer = None
    frame_count = 0
    for frame in frame_source(args.source):
        canvas, tensor = preprocess(frame, args.img_size, device, half)
        t0 = time.perf_counter()
        _da_mask, ll_mask = infer(model, tensor)
        dt_ms = (time.perf_counter() - t0) * 1000

        estimate, debug = estimate_lane_path(ll_mask.astype(np.uint8), H, grid, lookahead_m=args.lookahead)

        if logger:
            logger.log(infer_ms=dt_ms, estimate=estimate, fit_result=debug["fit"])

        combined = build_debug_frame(
            canvas, ll_mask, estimate, debug, grid, args.lookahead, H_inv, dt_ms, is_placeholder)

        frame_count += 1
        if args.save:
            if writer is None:
                h, w = combined.shape[:2]
                if str(args.save).lower().endswith((".jpg", ".png")):
                    cv2.imwrite(args.save, combined)
                    writer = "image_written"
                else:
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
            if writer not in (None, "image_written"):
                writer.write(combined)

        if not args.headless:
            cv2.imshow("lane fit debug (q to quit)", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        elif args.save and writer == "image_written":
            break

        if args.max_frames is not None and frame_count >= args.max_frames:
            break

    if isinstance(writer, cv2.VideoWriter):
        writer.release()
    if logger:
        logger.close()
        print(f"CSV 저장됨: {args.log_csv} ({frame_count}프레임)")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
