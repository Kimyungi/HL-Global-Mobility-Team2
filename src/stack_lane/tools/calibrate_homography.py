"""바닥에 놓은 실측 기준점으로 픽셀→vehicle frame 호모그래피를 구한다.

절차:
  1. capture_calibration_frame.py로 찍은 사진을 이 스크립트로 연다.
  2. 사진 속 각 마커를 마우스로 클릭한다.
  3. 클릭할 때마다 터미널에 그 마커의 실측 (x, y)[m]를 입력한다.
     x=차량 전방 방향 거리(+), y=좌측(+). 검증 전용으로만 쓸 점은
     끝에 v를 붙인다 (예: "3.0 1.85 v") — 이 점은 H 계산엔 안 쓰고
     계산된 H가 이 점을 얼마나 잘 맞추는지 검증하는 데만 쓴다.
  4. 최소 4점(권장: fit 8점 이상 + validation 3~4점) 다 찍은 뒤 'c'로 계산+저장.
     잘못 찍었으면 'u'로 마지막 점 취소.

좌표계(원점 정의, PROJECT_BRIEF.md §6 합의):
  원점(0,0) = 카메라 광학중심을 지면에 수직으로 투영한 점 (plumb line으로 표시).
  x축 = 차량 전방 진행 방향, y축 = 그 방향 기준 좌측(+).
  카메라 mounting yaw가 차량 중심선과 살짝 어긋나 있어도, 기준점을 "차량 전방
  방향" 기준으로 재기만 하면 그 어긋남은 H가 알아서 흡수한다 — 별도 보정 불필요.

사용:
  python3 calibrate_homography.py --image ../captures/calib_frame.png
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

# 창 이름은 ASCII만 — 유니코드(—) 포함 시 OpenCV Qt 백엔드가 창 핸들 조회에
# 실패해 setMouseCallback이 NULL window 에러를 낸다 (2026-08-11 현장에서 확인).
WINDOW = "calibration - click a marker, then answer in terminal"
DEFAULT_OUT = Path(__file__).resolve().parent.parent / "config" / "homography.json"
DEFAULT_BEV_PREVIEW = Path(__file__).resolve().parent.parent / "config" / "homography_bev_preview.png"


class Picker:
    def __init__(self, image: np.ndarray):
        self.image = image
        self.display = image.copy()
        self.points: list[dict] = []  # {u, v, x, y, validation}

    def draw(self) -> None:
        self.display = self.image.copy()
        for i, p in enumerate(self.points):
            color = (0, 165, 255) if p["validation"] else (0, 255, 0)
            cv2.drawMarker(self.display, (int(p["u"]), int(p["v"])), color,
                            markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
            cv2.putText(self.display, f'{i}:({p["x"]:.2f},{p["y"]:.2f})',
                        (int(p["u"]) + 8, int(p["v"]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def on_click(self, event, u, v, flags, userdata) -> None:  # noqa: ANN001
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        raw = input(
            f'점 {len(self.points)} 픽셀=({u},{v}). 실제 좌표 "x y" [m] 입력'
            f' (검증용이면 끝에 v, 예: "3.0 1.85 v"), 빈 입력이면 취소: '
        ).strip()
        if not raw:
            print("취소됨 — 다시 클릭하세요")
            return
        parts = raw.split()
        validation = False
        if parts and parts[-1].lower() == "v":
            validation = True
            parts = parts[:-1]
        if len(parts) != 2:
            print("형식 오류. 예: '3.0 1.85' 또는 '3.0 1.85 v' — 다시 클릭하세요")
            return
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            print("숫자로 입력하세요 — 다시 클릭하세요")
            return
        self.points.append({"u": float(u), "v": float(v), "x": x, "y": y, "validation": validation})
        self.draw()
        n_val = sum(p["validation"] for p in self.points)
        print(f"  -> 등록됨 (총 {len(self.points)}점, 검증용 {n_val}개)")


def compute_homography(points: list[dict]) -> np.ndarray:
    fit = [p for p in points if not p["validation"]]
    val = [p for p in points if p["validation"]]
    if len(fit) < 4:
        raise ValueError(f"호모그래피 계산엔 fit용 최소 4점 필요 (현재 {len(fit)}점)")

    src = np.array([[p["u"], p["v"]] for p in fit], dtype=np.float64)
    dst = np.array([[p["x"], p["y"]] for p in fit], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None:
        raise RuntimeError("호모그래피 계산 실패 — 점 배치를 확인하세요 (한 직선 위에 몰리지 않았는지 등)")

    def report(pts: list[dict], label: str) -> None:
        if not pts:
            return
        px = np.array([[p["u"], p["v"]] for p in pts], dtype=np.float64).reshape(-1, 1, 2)
        pred = cv2.perspectiveTransform(px, H).reshape(-1, 2)
        gt = np.array([[p["x"], p["y"]] for p in pts], dtype=np.float64)
        err = np.linalg.norm(pred - gt, axis=1)
        print(f"\n[{label}] {len(pts)}점 재투영 오차 [m]:")
        for p, e, pr in zip(pts, err, pred):
            print(f"  실측=({p['x']:.2f},{p['y']:.2f})  예측=({pr[0]:.2f},{pr[1]:.2f})  오차={e:.3f}")
        print(f"  RMS={np.sqrt((err ** 2).mean()):.3f}  max={err.max():.3f}")

    report(fit, "FIT (H 계산에 사용 — 큰 오차가 있으면 오타 의심)")
    report(val, "VALIDATION (H 계산에 미사용 — 실제 정확도 지표)")
    if not val:
        print("\n주의: 검증용 점이 없음 — fit 오차만으로는 과적합 여부를 알 수 없음. "
              "다음엔 몇 개는 'v'로 남겨두는 걸 권장.")

    return H


def save_bev_preview(image: np.ndarray, H: np.ndarray, out_path: Path,
                      scale_px_per_m: float = 100.0, x_max_m: float = 6.0,
                      lateral_half_m: float = 3.0) -> None:
    bev_w = int(2 * lateral_half_m * scale_px_per_m)
    bev_h = int(x_max_m * scale_px_per_m)
    # world(x,y)[m] -> bev pixel(col,row): x=전방 클수록 위(row 작음), y=좌측 클수록 왼쪽(col 작음)
    A = np.array([
        [0.0, -scale_px_per_m, scale_px_per_m * lateral_half_m],
        [-scale_px_per_m, 0.0, scale_px_per_m * x_max_m],
        [0.0, 0.0, 1.0],
    ])
    M = A @ H  # 원본 픽셀 -> world -> bev 픽셀 합성
    bev = cv2.warpPerspective(image, M, (bev_w, bev_h))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), bev)
    print(f"BEV 미리보기 저장됨: {out_path} (지면이 평평하면 바닥 무늬/마커 격자가 곧게 나와야 정상)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", required=True, help="capture_calibration_frame.py로 찍은 사진 경로")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="계산된 H 저장 경로 (json)")
    parser.add_argument("--bev-preview", default=str(DEFAULT_BEV_PREVIEW))
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise FileNotFoundError(args.image)

    picker = Picker(image)
    picker.draw()
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, picker.on_click)

    print(__doc__)
    print("창에서 마커를 클릭하세요. 'c'=계산+저장, 'u'=마지막 점 취소, 'q'=종료(저장 안 함)")

    while True:
        cv2.imshow(WINDOW, picker.display)
        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            break
        if key == ord("u") and picker.points:
            removed = picker.points.pop()
            picker.draw()
            print(f"제거됨: {removed}")
        if key == ord("c"):
            try:
                H = compute_homography(picker.points)
            except Exception as exc:  # noqa: BLE001
                print(f"오류: {exc}")
                continue

            out_path = Path(args.out)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump({
                    "H": H.tolist(),
                    "convention": ("world = normalize(H @ [u, v, 1]^T); "
                                   "x=전방(+), y=좌측(+), 원점=카메라 지면 투영점"),
                    "source_image": str(Path(args.image).resolve()),
                    "points": picker.points,
                    "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                }, f, ensure_ascii=False, indent=2)
            print(f"\n저장됨: {out_path}")
            save_bev_preview(image, H, Path(args.bev_preview))
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
