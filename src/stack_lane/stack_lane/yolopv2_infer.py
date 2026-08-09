"""YOLOPv2 TorchScript 추론 — ROS 무의존, tools/ 스크립트와 실제 파이프라인이 공유.

전처리/후처리는 YOLOPv2 원저자 공식 demo.py/utils.py 로직을 그대로 이식했다
(letterbox, seg/ll 크롭 [12:372]는 img-size=640 기준 official 구현 고정값 —
img-size를 바꾸면 이 크롭 범위도 같이 바꿔야 함).
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F

DEFAULT_WEIGHTS = Path(__file__).resolve().parent.parent / "models" / "yolopv2.pt"


def resolve_device(device_arg: str) -> tuple[torch.device, bool]:
    if device_arg.lower() == "cpu":
        return torch.device("cpu"), False
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA 요청됐지만 torch.cuda.is_available() == False. --device cpu 사용")
    return torch.device(f"cuda:{device_arg}"), True


def load_model(weights, device: torch.device, half: bool):
    model = torch.jit.load(str(weights))
    model = model.to(device)
    if half:
        model.half()
    model.eval()
    return model


def letterbox(img: np.ndarray, new_shape: int = 640, color=(114, 114, 114), stride: int = 32) -> np.ndarray:
    shape = img.shape[:2]
    new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw = (new_shape[1] - new_unpad[0]) % stride
    dh = (new_shape[0] - new_unpad[1]) % stride
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    return cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)


def preprocess(frame_bgr: np.ndarray, img_size: int, device: torch.device, half: bool):
    canvas = cv2.resize(frame_bgr, (1280, 720), interpolation=cv2.INTER_LINEAR)
    net_in = letterbox(canvas, img_size)
    net_in = net_in[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
    net_in = np.ascontiguousarray(net_in)
    tensor = torch.from_numpy(net_in).to(device)
    tensor = tensor.half() if half else tensor.float()
    tensor /= 255.0
    if tensor.ndimension() == 3:
        tensor = tensor.unsqueeze(0)
    return canvas, tensor


def driving_area_mask(seg: torch.Tensor) -> np.ndarray:
    cropped = seg[:, :, 12:372, :]
    up = F.interpolate(cropped, scale_factor=2, mode="bilinear")
    _, mask = torch.max(up, 1)
    return mask.int().squeeze().cpu().numpy()


def lane_line_mask(ll: torch.Tensor) -> np.ndarray:
    cropped = ll[:, :, 12:372, :]
    up = F.interpolate(cropped, scale_factor=2, mode="bilinear")
    mask = torch.round(up).squeeze(1)
    return mask.int().squeeze().cpu().numpy()


def infer(model, tensor: torch.Tensor):
    with torch.no_grad():
        (_, _), seg, ll = model(tensor)
    return driving_area_mask(seg), lane_line_mask(ll)
