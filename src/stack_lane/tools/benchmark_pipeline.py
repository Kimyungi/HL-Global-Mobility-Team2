"""전체 stack_lane 파이프라인(YOLOPv2 추론 + BEV 워프 + 슬라이딩 윈도우) 레이턴시 측정.

REQUIREMENTS.md의 100ms 예산(카메라 주기) 안에 실제로 들어오는지 확인하는 용도.
추론 시간만 쟀던 이전 벤치마크(PROJECT_BRIEF.md §9)와 달리 BEV 워프·필터·슬라이딩
윈도우·다항식 피팅까지 포함한 프레임당 총 소요시간을 잰다.

주의: 차선이 거의 안 보이는(mode=none) 조건에서 재면 lane_path 단계가 일찍
끝나버려 최선의 경우만 측정됨 — 실제 both/편측 검출이 있는 영상으로도 같이
재는 걸 권장 (--source에 저장된 mp4를 넣으면 됨).

사용:
  python3 benchmark_pipeline.py --source oak --device 0 --frames 200
  python3 benchmark_pipeline.py --source ~/lane_debug_video.mp4 --device 0 --frames 200
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from visualize_yolopv2 import frame_source  # noqa: E402

from stack_lane.bev import BevGrid, load_homography  # noqa: E402
from stack_lane.lane_path import estimate_lane_path  # noqa: E402
from stack_lane.yolopv2_infer import (  # noqa: E402
    DEFAULT_WEIGHTS,
    infer,
    load_model,
    preprocess,
    resolve_device,
)


def report(name: str, values: list[float]) -> None:
    arr = np.array(values)
    print(f"{name:12s}: mean={arr.mean():6.2f}ms  median={np.median(arr):6.2f}ms  "
          f"p95={np.percentile(arr, 95):6.2f}ms  max={arr.max():6.2f}ms")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    parser.add_argument("--source", default="oak")
    parser.add_argument("--img-size", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--lookahead", type=float, default=3.0)
    parser.add_argument("--homography", default=None)
    parser.add_argument("--frames", type=int, default=200, help="워밍업 제외하고 측정할 프레임 수")
    parser.add_argument("--warmup", type=int, default=10)
    args = parser.parse_args()

    device, half = resolve_device(args.device)
    model = load_model(args.weights, device, half)
    H, is_placeholder, _meta = load_homography(args.homography)
    grid = BevGrid()

    infer_ms: list[float] = []
    lane_path_ms: list[float] = []
    total_ms: list[float] = []
    mode_counts: dict[str, int] = {}
    count = 0

    for frame in frame_source(args.source):
        canvas, tensor = preprocess(frame, args.img_size, device, half)

        t0 = time.perf_counter()
        _da_mask, ll_mask = infer(model, tensor)
        t1 = time.perf_counter()
        estimate, _debug = estimate_lane_path(ll_mask.astype(np.uint8), H, grid, lookahead_m=args.lookahead)
        t2 = time.perf_counter()

        count += 1
        if count <= args.warmup:
            continue

        infer_ms.append((t1 - t0) * 1000)
        lane_path_ms.append((t2 - t1) * 1000)
        total_ms.append((t2 - t0) * 1000)
        mode_counts[estimate.mode] = mode_counts.get(estimate.mode, 0) + 1

        if len(total_ms) >= args.frames:
            break

    print(f"\n측정 프레임 수: {len(total_ms)} (워밍업 {args.warmup}프레임 제외)")
    print(f"mode 분포: {mode_counts}")
    if is_placeholder:
        print("[참고] placeholder 호모그래피 사용 중 (BEV 워프 연산량 자체는 실측 H와 동일)")
    report("infer", infer_ms)
    report("lane_path", lane_path_ms)
    report("total", total_ms)
    over_budget = sum(1 for t in total_ms if t > 100)
    print(f"\n100ms 초과 프레임: {over_budget}/{len(total_ms)} ({over_budget / len(total_ms) * 100:.1f}%)")


if __name__ == "__main__":
    main()
