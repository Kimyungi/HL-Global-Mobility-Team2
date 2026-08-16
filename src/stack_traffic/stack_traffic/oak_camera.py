"""DepthAI 2.x 기반 OAK-D RGB + RGB 정렬 depth 입력."""

from __future__ import annotations

from datetime import timedelta
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    dai = None


class OakRgbdCamera:
    """동기화된 RGB 프레임과 millimetre depth 프레임을 읽는다."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: float,
        depth_enabled: bool,
        depth_confidence_threshold: int,
        depth_left_right_check: bool,
        depth_subpixel: bool,
        depth_median_filter_size: int,
        depth_decimation_factor: int,
        depth_speckle_filter: bool,
        depth_spatial_filter: bool,
        depth_temporal_filter: bool,
        minimum_depth_m: float,
        maximum_depth_m: float,
        mxid: str = "",
        max_usb_speed: str = "super",
    ) -> None:
        if dai is None:
            raise RuntimeError(
                "OAK 모드에는 depthai가 필요합니다: "
                "python3 -m pip install 'depthai>=2.30,<3.0'"
            )

        self.depth_enabled = depth_enabled
        self.pipeline = build_oak_pipeline(
            width,
            height,
            fps,
            depth_confidence_threshold=depth_confidence_threshold,
            depth_left_right_check=depth_left_right_check,
            depth_subpixel=depth_subpixel,
            depth_median_filter_size=depth_median_filter_size,
            depth_decimation_factor=depth_decimation_factor,
            depth_speckle_filter=depth_speckle_filter,
            depth_spatial_filter=depth_spatial_filter,
            depth_temporal_filter=depth_temporal_filter,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
            depth_enabled=depth_enabled,
        )
        # USB 링크 속도 상한 — 'high'면 USB 2.0으로 강제해 SuperSpeed(5Gbps) 신호
        # 자체를 없앤다. OAK-D의 USB3 방사 잡음이 GNSS L1(1575MHz)을 덮어 RTK
        # FIXED를 죽이는 것이 2026-08-14 실측으로 확인됐다(같은 안테나 위치에서
        # C/N0 39dB↔22dB, 16.5dB 차). stack_lane도 동일 대책(usb_speed) 적용.
        # ⚠ USB2 대역폭 ~40MB/s — oak_fps/해상도가 크면 프레임이 밀린다.
        speed_name = str(max_usb_speed or "super").strip().upper()
        max_speed = None
        if speed_name and speed_name != "SUPER":
            max_speed = getattr(dai.UsbSpeed, speed_name, None)
        # MxID 핀닝 — 카메라 2대(차선/신호등) 운용 시 필수. 없으면 어느 노드가
        # 어느 카메라를 잡을지 부팅 순서에 따라 뒤바뀐다 (CLAUDE.md §6).
        device_info = dai.DeviceInfo(mxid) if mxid else None
        try:
            if device_info is not None and max_speed is not None:
                self.device = dai.Device(self.pipeline, device_info, max_speed)
            elif device_info is not None:
                self.device = dai.Device(self.pipeline, device_info)
            elif max_speed is not None:
                self.device = dai.Device(self.pipeline, max_speed)
            else:
                self.device = dai.Device(self.pipeline)
        except Exception as error:
            raise RuntimeError(
                "OAK-D Pro를 열 수 없습니다. USB 연결과 권한을 확인하세요"
                f"(mxid={mxid or '미지정'}, max_usb_speed={speed_name}): "
                f"{error}"
            ) from error
        self.usb_speed = str(self.device.getUsbSpeed()).split(".")[-1]
        self.last_read_status = "starting"
        self.depth_native_shape: Optional[Tuple[int, int]] = None
        self.depth_resized = False
        self.queue = self.device.getOutputQueue(
            name="rgbd" if self.depth_enabled else "rgb",
            maxSize=1,
            blocking=False,
        )

    def read(
        self,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """새 동기 RGB/depth 묶음이 있으면 반환한다."""
        try:
            message_group = self.queue.tryGet()
        except Exception:
            self.last_read_status = "error"
            return False, None, None
        if message_group is None:
            self.last_read_status = "empty"
            return False, None, None

        try:
            if self.depth_enabled:
                rgb_message = message_group["rgb"]
                depth_message = message_group["depth"]
                frame = rgb_message.getCvFrame()
                depth_mm = depth_message.getFrame()
            else:
                frame = message_group.getCvFrame()
                depth_mm = None
        except (KeyError, RuntimeError, TypeError, AttributeError):
            self.last_read_status = "error"
            return False, None, None

        if frame is None or (self.depth_enabled and depth_mm is None):
            self.last_read_status = "error"
            return False, None, None
        if depth_mm is not None and frame.shape[:2] != depth_mm.shape[:2]:
            self.depth_native_shape = tuple(depth_mm.shape[:2])
            frame_ratio = frame.shape[1] / float(frame.shape[0])
            depth_ratio = depth_mm.shape[1] / float(depth_mm.shape[0])
            if abs(frame_ratio - depth_ratio) > 0.02:
                self.last_read_status = "shape_error"
                return False, None, None
            depth_mm = cv2.resize(
                depth_mm,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
            self.depth_resized = True
        elif depth_mm is not None:
            self.depth_native_shape = tuple(depth_mm.shape[:2])
            self.depth_resized = False
        self.last_read_status = "ok"
        return True, frame, depth_mm

    def release(self) -> None:
        """DepthAI 장치를 닫는다."""
        if hasattr(self, "device"):
            self.device.close()


def build_oak_pipeline(
    width: int,
    height: int,
    fps: float,
    depth_confidence_threshold: int = 245,
    depth_left_right_check: bool = True,
    depth_subpixel: bool = True,
    depth_median_filter_size: int = 7,
    depth_decimation_factor: int = 1,
    depth_speckle_filter: bool = True,
    depth_spatial_filter: bool = True,
    depth_temporal_filter: bool = True,
    minimum_depth_m: float = 0.3,
    maximum_depth_m: float = 20.0,
    depth_enabled: bool = True,
):
    """OAK-D Pro용 RGB 또는 RGB 정렬 stereo depth 파이프라인을 만든다."""
    if dai is None:
        raise RuntimeError("depthai가 설치되어 있지 않습니다.")

    pipeline = dai.Pipeline()
    color = pipeline.create(dai.node.ColorCamera)
    output = pipeline.create(dai.node.XLinkOut)

    color.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    color.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P
    )
    color.setPreviewSize(width, height)
    color.setInterleaved(False)
    color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    color.setFps(fps)

    output.input.setBlocking(False)
    output.input.setQueueSize(1)
    if not depth_enabled:
        output.setStreamName("rgb")
        color.preview.link(output.input)
        return pipeline

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)

    for camera, socket in (
        (left, dai.CameraBoardSocket.CAM_B),
        (right, dai.CameraBoardSocket.CAM_C),
    ):
        camera.setBoardSocket(socket)
        camera.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_800_P
        )
        camera.setFps(fps)

    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.DEFAULT
    )
    median_filters = {
        0: dai.MedianFilter.MEDIAN_OFF,
        3: dai.MedianFilter.KERNEL_3x3,
        5: dai.MedianFilter.KERNEL_5x5,
        7: dai.MedianFilter.KERNEL_7x7,
    }
    stereo.setLeftRightCheck(depth_left_right_check)
    stereo.setSubpixel(depth_subpixel)
    stereo.initialConfig.setConfidenceThreshold(
        depth_confidence_threshold
    )
    stereo.initialConfig.setMedianFilter(
        median_filters[depth_median_filter_size]
    )
    stereo_config = stereo.initialConfig.get()
    stereo_config.postProcessing.speckleFilter.enable = (
        depth_speckle_filter
    )
    stereo_config.postProcessing.spatialFilter.enable = (
        depth_spatial_filter
    )
    stereo_config.postProcessing.spatialFilter.holeFillingRadius = 2
    stereo_config.postProcessing.spatialFilter.numIterations = 1
    stereo_config.postProcessing.temporalFilter.enable = (
        depth_temporal_filter
    )
    # decimation은 RGB 신호등 영상이 아니라 depth 계산량만 줄인다.
    # setOutputSize로 최종 depth는 RGB 크기에 다시 정렬된다.
    stereo_config.postProcessing.decimationFilter.decimationFactor = (
        depth_decimation_factor
    )
    stereo_config.postProcessing.thresholdFilter.minRange = int(
        minimum_depth_m * 1000.0
    )
    stereo_config.postProcessing.thresholdFilter.maxRange = int(
        maximum_depth_m * 1000.0
    )
    stereo.initialConfig.set(stereo_config)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(width, height)
    stereo.setOutputKeepAspectRatio(True)

    sync.setSyncThreshold(timedelta(milliseconds=50))
    output.setStreamName("rgbd")

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    color.preview.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])
    sync.out.link(output.input)
    return pipeline
