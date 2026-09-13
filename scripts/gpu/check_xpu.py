#!/usr/bin/env python3
"""Require driver access and real XPU computation, then test the lane model."""
import glob
import os
from pathlib import Path
import platform
import time

cmdline = Path('/proc/cmdline').read_text().strip()
print('Kernel:', platform.release(), flush=True)
print('Kernel command line:', cmdline, flush=True)
if 'nomodeset' in cmdline.split():
    raise SystemExit('nomodeset is active; reboot normally before XPU verification.')
nodes = glob.glob('/dev/dri/renderD*')
print('Render devices:', nodes, flush=True)
if not any(os.access(p, os.R_OK | os.W_OK) for p in nodes):
    raise SystemExit('No accessible render device: check Xe boot log and render group membership.')
import torch
print('Torch:', torch.__version__, flush=True)
if not torch.xpu.is_available():
    raise SystemExit('XPU unavailable: driver/runtime still needs attention.')
print('GPU:', torch.xpu.get_device_name(0), flush=True)
x = torch.arange(16, device='xpu', dtype=torch.float32)
assert (x * x).sum().item() == 1240
print('XPU computation passed.', flush=True)
model_path = Path.home() / 'FMA_ws/src/stack_lane/models/yolopv2.pt'
model = torch.jit.load(str(model_path), map_location='xpu').eval().half()
x = torch.zeros((1, 3, 640, 640), device='xpu', dtype=torch.float16)
with torch.inference_mode():
    for _ in range(3):
        model(x)
    torch.xpu.synchronize()
    start = time.monotonic()
    for _ in range(10):
        model(x)
    torch.xpu.synchronize()
print(f'Lane model: {(time.monotonic() - start) * 100:.1f} ms/inference (synthetic input; excludes camera/ROS).')
