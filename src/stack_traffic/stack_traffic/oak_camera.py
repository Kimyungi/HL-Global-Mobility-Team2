"""DepthAI 2.x 기반 OAK-D RGB + RGB 정렬 depth 입력."""

from __future__ import annotations

from datetime import timedelta
import time
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    dai = None


OAK_OPEN_MAX_ATTEMPTS = 3
OAK_OPEN_RETRY_INTERVAL_SEC = 2.0


def normalize_oak_mxid(value: object) -> str:
    """ROS/CLI에서 받은 OAK MxID를 한 가지 형식으로 정규화한다."""
    return str(value).strip()


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
    ) -> None:
        if dai is None:
            raise RuntimeError(
                "OAK 모드에는 depthai가 필요합니다: "
                "python3 -m pip install 'depthai>=2.30,<3.0'"
            )

        self.depth_enabled = depth_enabled
        self.requested_mxid = normalize_oak_mxid(mxid)
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
        try:
            self.device = open_oak_device(
                self.pipeline,
                self.requested_mxid,
            )
        except Exception as error:
            requested = (
                f" (oak_mxid={self.requested_mxid})"
                if self.requested_mxid
                else ""
            )
            raise RuntimeError(
                "OAK-D Pro를 열 수 없습니다"
                f"{requested}. USB 연결, 권한, MxID를 확인하세요: "
                f"{error}"
            ) from error
        try:
            self.mxid = str(self.device.getMxId())
        except (AttributeError, RuntimeError):
            self.mxid = self.requested_mxid or "unknown"
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


def open_oak_device(pipeline, mxid: str = ""):
    """OAK 장치를 결정적으로 열고 일시적인 재열거 상태만 재시도한다."""
    if dai is None:
        raise RuntimeError("depthai가 설치되어 있지 않습니다.")

    requested_mxid = normalize_oak_mxid(mxid)
    last_error: Optional[Exception] = None

    for attempt in range(1, OAK_OPEN_MAX_ATTEMPTS + 1):
        if requested_mxid:
            try:
                device_info = dai.DeviceInfo(requested_mxid)
                return dai.Device(pipeline, device_info)
            except Exception as error:
                last_error = error
        else:
            # 이미 다른 프로세스가 연 장치도 포함해야 부팅 순서와
            # 무관하게 다중 OAK 환경을 감지할 수 있다. 재시도마다
            # 목록을 새로 받아 BOOTED/재열거 중인 스냅샷을 재사용하지 않는다.
            try:
                device_infos = list(dai.Device.getAllConnectedDevices())
            except Exception as error:
                last_error = error
            else:
                if len(device_infos) > 1:
                    connected_mxids = [
                        str(info.getMxId()) for info in device_infos
                    ]
                    raise RuntimeError(
                        "OAK-D가 여러 대 연결되어 있어 자동 선택할 수 "
                        "없습니다. stack_traffic의 oak_mxid를 지정하세요. "
                        f"connected_mxids={connected_mxids}"
                    )
                if not device_infos:
                    last_error = RuntimeError(
                        "연결된 OAK-D가 열거되지 않았습니다."
                    )
                else:
                    try:
                        return dai.Device(pipeline, device_infos[0])
                    except Exception as error:
                        last_error = error

        if attempt < OAK_OPEN_MAX_ATTEMPTS:
            time.sleep(OAK_OPEN_RETRY_INTERVAL_SEC)

    target = (
        f"oak_mxid={requested_mxid}"
        if requested_mxid
        else "빈 oak_mxid의 단일 OAK-D"
    )
    raise RuntimeError(
        f"{target} 연결을 {OAK_OPEN_MAX_ATTEMPTS}회 시도했지만 "
        f"실패했습니다: {last_error}"
    ) from last_error


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
