"""DepthAI 2.x/3.x OAK-D RGB + RGB 정렬 depth 입력."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
import re
import time
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    dai = None


OAK_OPEN_MAX_ATTEMPTS = 3
OAK_OPEN_RETRY_INTERVAL_SEC = 2.0
VALID_OAK_USB_SPEEDS = ("high", "super")
USB2_SAFE_PAYLOAD_BYTES_PER_SEC = 36_000_000
MINIMUM_DEPTHAI_VERSION = {2: (2, 30), 3: (3, 6)}


class OakConfigurationError(RuntimeError):
    """재시도로 해결할 수 없는 OAK 선택/USB 설정 오류."""


@dataclass
class _OakSession:
    """DepthAI API 세대와 무관한 열린 카메라 세션."""

    api_major: int
    device: object
    pipeline: object
    queue: object
    mxid: str
    usb_speed: str


def normalize_oak_mxid(value: object) -> str:
    """ROS/CLI에서 받은 OAK MxID를 한 가지 형식으로 정규화한다."""
    return str(value).strip()


def normalize_oak_usb_speed(value: object) -> str:
    """허용된 USB 링크 상한을 엄격하게 정규화한다."""
    normalized = str(value).strip().lower()
    if normalized not in VALID_OAK_USB_SPEEDS:
        allowed = ", ".join(VALID_OAK_USB_SPEEDS)
        raise ValueError(
            f"oak_usb_speed는 {allowed} 중 하나여야 합니다: {value!r}"
        )
    return normalized


def depthai_api_major() -> int:
    """장치를 열지 않고 설치된 DepthAI API 세대를 판별한다."""
    if dai is None:
        raise RuntimeError("depthai가 설치되어 있지 않습니다.")
    return 3 if hasattr(dai.Pipeline, "start") else 2


def validate_depthai_version() -> tuple[int, tuple[int, ...]]:
    """지원한 API 세대의 최소 DepthAI binding 버전을 강제한다."""
    api_major = depthai_api_major()
    raw_version = getattr(dai, "__version__", "")
    if not isinstance(raw_version, str):
        raise RuntimeError("DepthAI runtime 버전을 확인할 수 없습니다.")
    numbers = tuple(int(value) for value in re.findall(r"\d+", raw_version))
    minimum = MINIMUM_DEPTHAI_VERSION.get(api_major)
    version_major = numbers[0] if numbers else None
    if (
        minimum is None
        or version_major != api_major
        or numbers[:len(minimum)] < minimum
    ):
        required = ".".join(str(value) for value in minimum or ())
        raise RuntimeError(
            "지원하지 않는 DepthAI runtime입니다: "
            f"detected={raw_version or 'unknown'}, API={api_major}.x, "
            f"required>={required}, runtime/API major 일치 필요"
        )
    return api_major, numbers


def estimate_oak_host_bandwidth_bytes_per_sec(
    width: int,
    height: int,
    fps: float,
    depth_enabled: bool,
) -> float:
    """호스트로 전송하는 비압축 BGR(+uint16 depth) payload를 계산한다."""
    bytes_per_pixel = 3 + (2 if depth_enabled else 0)
    return float(width * height) * float(fps) * bytes_per_pixel


def validate_oak_usb_bandwidth(
    *,
    width: int,
    height: int,
    fps: float,
    depth_enabled: bool,
    usb_speed: str,
) -> None:
    """USB2 실효 대역폭을 넘는 설정은 장치를 열기 전에 거부한다."""
    if usb_speed != "high":
        return
    payload = estimate_oak_host_bandwidth_bytes_per_sec(
        width,
        height,
        fps,
        depth_enabled,
    )
    if payload <= USB2_SAFE_PAYLOAD_BYTES_PER_SEC:
        return
    raise ValueError(
        "USB2(HIGH) 안전 대역폭을 넘는 OAK 설정입니다: "
        f"{width}x{height}@{fps:g}, "
        f"depth={depth_enabled}, payload={payload / 1_000_000:.1f}MB/s. "
        "RGB-only 1280x720@10 또는 RGBD 640x360@10을 사용하세요."
    )


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
        usb_speed: str = "super",
    ) -> None:
        if dai is None:
            raise RuntimeError(
                "OAK 모드에는 depthai가 필요합니다: "
                "python3 -m pip install 'depthai>=2.30'"
            )

        api_major, _depthai_version = validate_depthai_version()

        self.depth_enabled = depth_enabled
        self.requested_mxid = normalize_oak_mxid(mxid)
        self.requested_usb_speed = normalize_oak_usb_speed(usb_speed)
        validate_oak_usb_bandwidth(
            width=width,
            height=height,
            fps=fps,
            depth_enabled=depth_enabled,
            usb_speed=self.requested_usb_speed,
        )
        pipeline_kwargs = {
            "width": width,
            "height": height,
            "fps": fps,
            "depth_confidence_threshold": depth_confidence_threshold,
            "depth_left_right_check": depth_left_right_check,
            "depth_subpixel": depth_subpixel,
            "depth_median_filter_size": depth_median_filter_size,
            "depth_decimation_factor": depth_decimation_factor,
            "depth_speckle_filter": depth_speckle_filter,
            "depth_spatial_filter": depth_spatial_filter,
            "depth_temporal_filter": depth_temporal_filter,
            "minimum_depth_m": minimum_depth_m,
            "maximum_depth_m": maximum_depth_m,
            "depth_enabled": depth_enabled,
        }
        try:
            if api_major >= 3:
                session = _open_oak_v3_session(
                    requested_mxid=self.requested_mxid,
                    requested_usb_speed=self.requested_usb_speed,
                    **pipeline_kwargs,
                )
            else:
                session = _open_oak_v2_session(
                    requested_mxid=self.requested_mxid,
                    requested_usb_speed=self.requested_usb_speed,
                    **pipeline_kwargs,
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
        self.api_major = session.api_major
        self.device = session.device
        self.pipeline = session.pipeline
        self.queue = session.queue
        self.mxid = session.mxid
        self.usb_speed = session.usb_speed.upper()
        self.last_read_status = "starting"
        self.depth_native_shape: Optional[Tuple[int, int]] = None
        self.depth_resized = False

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
        """연결된 OAK 장치를 닫는다."""
        queue = getattr(self, "queue", None)
        pipeline = getattr(self, "pipeline", None)
        device = getattr(self, "device", None)
        self.queue = None
        self.pipeline = None
        self.device = None
        _close_oak_resources(
            queue=queue,
            pipeline=pipeline,
            device=device,
            api_major=getattr(self, "api_major", 2),
        )


def open_oak_device(
    pipeline,
    mxid: str = "",
    usb_speed: str = "super",
):
    """MxID를 한 번 정규화한 뒤 OAK 장치를 결정적으로 연다."""
    if dai is None:
        raise RuntimeError("depthai가 설치되어 있지 않습니다.")

    requested_mxid = normalize_oak_mxid(mxid)
    requested_usb_speed = normalize_oak_usb_speed(usb_speed)
    return _open_oak_device_normalized(
        pipeline,
        requested_mxid,
        requested_usb_speed,
    )


def _open_oak_device_normalized(
    pipeline,
    requested_mxid: str,
    requested_usb_speed: str = "super",
):
    """이미 정규화된 MxID로 OAK를 열고 일시 오류만 재시도한다."""
    usb_speed = _usb_speed_enum(requested_usb_speed)

    def open_device(device_info, expected_mxid):
        device = dai.Device(pipeline, device_info, usb_speed)
        try:
            _verify_opened_device(
                device,
                expected_mxid,
                requested_usb_speed,
            )
        except Exception:
            _close_oak_resources(device=device, api_major=2)
            raise
        return device

    return _open_selected_with_retry(requested_mxid, open_device)


def _open_selected_with_retry(
    requested_mxid: str,
    opener: Callable[[object, str], object],
):
    """매 시도마다 장치 정보를 새로 얻어 지정 장치만 연다."""
    last_error: Optional[Exception] = None

    for attempt in range(1, OAK_OPEN_MAX_ATTEMPTS + 1):
        if requested_mxid:
            try:
                device_info = dai.DeviceInfo(requested_mxid)
                return opener(device_info, requested_mxid)
            except OakConfigurationError:
                raise
            except (RuntimeError, OSError) as error:
                last_error = error
        else:
            # 이미 다른 프로세스가 연 장치도 포함해야 부팅 순서와
            # 무관하게 다중 OAK 환경을 감지할 수 있다. 재시도마다
            # 목록을 새로 받아 BOOTED/재열거 중인 스냅샷을 재사용하지 않는다.
            try:
                device_infos = _get_all_connected_devices()
            except (RuntimeError, OSError) as error:
                last_error = error
            else:
                if len(device_infos) > 1:
                    connected_mxids = [
                        _get_device_id(info) for info in device_infos
                    ]
                    raise OakConfigurationError(
                        "OAK-D가 여러 대 연결되어 있어 자동 선택할 수 "
                        "없습니다. stack_traffic의 oak_mxid를 지정하세요. "
                        f"connected_mxids={connected_mxids}"
                    )
                if not device_infos:
                    last_error = RuntimeError(
                        "연결된 OAK-D가 열거되지 않았습니다."
                    )
                else:
                    expected_mxid = _get_device_id(device_infos[0])
                    if expected_mxid == "unknown":
                        raise OakConfigurationError(
                            "열거된 OAK-D의 MxID를 확인할 수 없어 자동 "
                            "선택을 거부합니다. oak_mxid를 명시하세요."
                        )
                    try:
                        return opener(device_infos[0], expected_mxid)
                    except OakConfigurationError:
                        raise
                    except (RuntimeError, OSError) as error:
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


def _get_all_connected_devices():
    """연결된 장치를 현재 API 세대에 맞게 열거한다."""
    getter = getattr(dai.Device, "getAllConnectedDevices", None)
    if getter is None:
        getter = dai.Device.getAllAvailableDevices
    return list(getter())


def _get_device_id(device_or_info) -> str:
    """OAK 장치의 MxID 또는 DeviceId를 한 형식으로 읽는다."""
    # v2의 표준 명칭인 getMxId를 먼저 사용한다. v3 DeviceInfo에는
    # getDeviceId만 있으므로 그때만 두 번째 경로로 넘어간다.
    for method_name in ("getMxId", "getDeviceId"):
        method = getattr(device_or_info, method_name, None)
        if method is None:
            continue
        try:
            value = str(method()).strip()
        except (AttributeError, RuntimeError, TypeError):
            continue
        if value:
            return value
    return "unknown"


def _get_usb_speed_name(device) -> str:
    try:
        return str(device.getUsbSpeed()).split(".")[-1].strip().lower()
    except (AttributeError, RuntimeError, TypeError):
        return "unknown"


def _usb_speed_enum(requested_usb_speed: str):
    normalized = normalize_oak_usb_speed(requested_usb_speed)
    return getattr(dai.UsbSpeed, normalized.upper())


def _verify_opened_device(
    device,
    expected_mxid: str,
    requested_usb_speed: str,
) -> tuple[str, str]:
    """다른 장치나 비의도 링크 속도로 열린 경우 즉시 거부한다."""
    connected_mxid = _get_device_id(device)
    if connected_mxid == "unknown":
        raise OakConfigurationError(
            "연결된 OAK-D의 실제 MxID를 확인할 수 없어 기동을 "
            "거부합니다."
        )
    if connected_mxid != expected_mxid:
        raise OakConfigurationError(
            "요청한 OAK-D와 실제 연결 장치가 다릅니다: "
            f"expected={expected_mxid}, connected={connected_mxid}"
        )
    actual_usb_speed = _get_usb_speed_name(device)
    if actual_usb_speed == "unknown":
        raise OakConfigurationError(
            "OAK-D의 실제 USB 링크 속도를 확인할 수 없어 기동을 "
            "거부합니다."
        )
    if requested_usb_speed == "high" and actual_usb_speed != "high":
        raise OakConfigurationError(
            "USB2(HIGH)를 요청했지만 실제 링크가 HIGH가 아닙니다: "
            f"actual={actual_usb_speed}. 케이블/포트와 lsusb -t를 "
            "확인하세요."
        )
    return connected_mxid, actual_usb_speed


def _verify_actual_usb_bandwidth(
    *,
    actual_usb_speed: str,
    width: int,
    height: int,
    fps: float,
    depth_enabled: bool,
) -> None:
    """협상된 실제 링크 속도로도 스트림 설정을 다시 검증한다."""
    if actual_usb_speed in ("super", "super_plus"):
        return
    if actual_usb_speed == "high":
        try:
            validate_oak_usb_bandwidth(
                width=width,
                height=height,
                fps=fps,
                depth_enabled=depth_enabled,
                usb_speed="high",
            )
        except ValueError as error:
            raise OakConfigurationError(str(error)) from error
        return
    raise OakConfigurationError(
        "지원하지 않는 실제 OAK-D USB 링크 속도입니다: "
        f"actual={actual_usb_speed}. HIGH(USB2) 이상이 필요합니다."
    )


def _close_oak_resources(
    *,
    queue=None,
    pipeline=None,
    device=None,
    api_major: int,
) -> None:
    """부분 기동 실패와 정상 종료에서 장치 자원을 안전하게 정리한다."""
    if queue is not None:
        try:
            queue.close()
        except (AttributeError, RuntimeError):
            pass
    if api_major >= 3 and pipeline is not None:
        try:
            pipeline.stop()
        except (AttributeError, RuntimeError):
            pass
        try:
            pipeline.wait()
        except (AttributeError, RuntimeError):
            pass
    if device is not None:
        try:
            device.close()
        except (AttributeError, RuntimeError):
            pass


def _open_oak_v2_session(
    *,
    requested_mxid: str,
    requested_usb_speed: str,
    **pipeline_kwargs,
) -> _OakSession:
    """버전 2 파이프라인·장치·큐 전체를 새로 만들어 재시도한다."""
    usb_speed = _usb_speed_enum(requested_usb_speed)

    def open_session(device_info, expected_mxid):
        pipeline = None
        device = None
        queue = None
        try:
            pipeline = build_oak_pipeline(**pipeline_kwargs)
            device = dai.Device(pipeline, device_info, usb_speed)
            connected_mxid, actual_usb_speed = _verify_opened_device(
                device,
                expected_mxid,
                requested_usb_speed,
            )
            _verify_actual_usb_bandwidth(
                actual_usb_speed=actual_usb_speed,
                width=pipeline_kwargs["width"],
                height=pipeline_kwargs["height"],
                fps=pipeline_kwargs["fps"],
                depth_enabled=pipeline_kwargs["depth_enabled"],
            )
            queue = device.getOutputQueue(
                name=(
                    "rgbd" if pipeline_kwargs["depth_enabled"] else "rgb"
                ),
                maxSize=1,
                blocking=False,
            )
            return _OakSession(
                2,
                device,
                pipeline,
                queue,
                connected_mxid,
                actual_usb_speed,
            )
        except Exception:
            _close_oak_resources(
                queue=queue,
                pipeline=pipeline,
                device=device,
                api_major=2,
            )
            raise

    return _open_selected_with_retry(requested_mxid, open_session)


def _open_oak_v3_session(
    *,
    requested_mxid: str,
    requested_usb_speed: str,
    **pipeline_kwargs,
) -> _OakSession:
    """버전 3 장치에 파이프라인을 묶어 열고 시작한다."""
    usb_speed = _usb_speed_enum(requested_usb_speed)

    def open_session(device_info, expected_mxid):
        device = dai.Device(device_info, usb_speed)
        pipeline = None
        queue = None
        try:
            connected_mxid, actual_usb_speed = _verify_opened_device(
                device,
                expected_mxid,
                requested_usb_speed,
            )
            _verify_actual_usb_bandwidth(
                actual_usb_speed=actual_usb_speed,
                width=pipeline_kwargs["width"],
                height=pipeline_kwargs["height"],
                fps=pipeline_kwargs["fps"],
                depth_enabled=pipeline_kwargs["depth_enabled"],
            )
            pipeline = dai.Pipeline(device)
            queue = _build_oak_v3_pipeline(
                pipeline,
                **pipeline_kwargs,
            )
            pipeline.start()
            return _OakSession(
                3,
                device,
                pipeline,
                queue,
                connected_mxid,
                actual_usb_speed,
            )
        except Exception:
            _close_oak_resources(
                queue=queue,
                pipeline=pipeline,
                device=device,
                api_major=3,
            )
            raise

    return _open_selected_with_retry(requested_mxid, open_session)


def _build_oak_v3_pipeline(
    pipeline,
    *,
    width: int,
    height: int,
    fps: float,
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
    depth_enabled: bool,
):
    """이미 선택된 DepthAI 3 장치에 RGB/RGBD 노드를 구성한다."""
    color = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_A
    )
    rgb_output = color.requestOutput(
        size=(width, height),
        type=dai.ImgFrame.Type.BGR888i,
        fps=float(fps),
    )
    if not depth_enabled:
        return rgb_output.createOutputQueue(maxSize=1, blocking=False)

    left = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_B
    )
    right = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_C
    )
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)
    left_output = left.requestOutput(
        size=(640, 400),
        fps=float(fps),
    )
    right_output = right.requestOutput(
        size=(640, 400),
        fps=float(fps),
    )

    _configure_stereo(
        stereo,
        width=width,
        height=height,
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
        align_with_input=True,
    )
    sync.setSyncThreshold(timedelta(milliseconds=50))
    left_output.link(stereo.left)
    right_output.link(stereo.right)
    rgb_output.link(stereo.inputAlignTo)
    rgb_output.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])
    return sync.out.createOutputQueue(maxSize=1, blocking=False)


def _configure_stereo(
    stereo,
    *,
    width: int,
    height: int,
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
    align_with_input: bool,
) -> None:
    """두 API 세대에 공통인 StereoDepth 필터를 구성한다."""
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
    # depthai 2.x: initialConfig.get()이 RawStereoDepthConfig을 반환하고
    # .set()으로 되돌려 써야 반영된다. depthai 3.6.1부터는 initialConfig
    # 자체가 postProcessing을 직접 노출하는 StereoDepthConfig이라 get/set
    # 왕복이 없다(오히려 .get 속성 자체가 없음, 실측 2026-09-01).
    has_get_set = hasattr(stereo.initialConfig, "get")
    stereo_config = stereo.initialConfig.get() if has_get_set else stereo.initialConfig
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
    stereo_config.postProcessing.decimationFilter.decimationFactor = (
        depth_decimation_factor
    )
    stereo_config.postProcessing.thresholdFilter.minRange = int(
        minimum_depth_m * 1000.0
    )
    stereo_config.postProcessing.thresholdFilter.maxRange = int(
        maximum_depth_m * 1000.0
    )
    if has_get_set:
        stereo.initialConfig.set(stereo_config)
    if not align_with_input:
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(width, height)
    stereo.setOutputKeepAspectRatio(True)


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

    _configure_stereo(
        stereo,
        width=width,
        height=height,
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
        align_with_input=False,
    )

    sync.setSyncThreshold(timedelta(milliseconds=50))
    output.setStreamName("rgbd")

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    color.preview.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])
    sync.out.link(output.input)
    return pipeline
