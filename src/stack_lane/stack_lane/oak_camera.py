"""
Deterministic OAK RGB camera setup for ``stack_lane``.

The vehicle carries two OAK cameras, so the lane process must open the
configured MxID and must not silently fall back to another device or USB link
speed.  DepthAI 2.x and 3.x use different construction orders; API detection
therefore inspects the ``Pipeline`` class without constructing a pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import time
from typing import Callable, Optional

try:
    import depthai as dai
except ImportError:
    dai = None


LANE_CAMERA_WIDTH = 1280
LANE_CAMERA_HEIGHT = 720
VALID_USB_SPEEDS = ("high", "super")
USB2_SAFE_PAYLOAD_BYTES_PER_SEC = 36_000_000
OPEN_MAX_ATTEMPTS = 4
OPEN_RETRY_INTERVAL_SEC = 8.0
MINIMUM_DEPTHAI_VERSION = {2: (2, 30), 3: (3, 6)}


class LaneOakConfigurationError(RuntimeError):
    """A deterministic camera selection or USB contract was violated."""


@dataclass
class _OakSession:
    api_major: int
    pipeline: object
    device: object
    queue: object


def normalize_usb_speed(value: object) -> str:
    """Return a strict, lower-case DepthAI USB speed name."""
    normalized = str(value).strip().lower()
    if normalized not in VALID_USB_SPEEDS:
        allowed = ", ".join(VALID_USB_SPEEDS)
        raise ValueError(
            f"usb_speed must be one of {allowed}: {value!r}"
        )
    return normalized


def depthai_api_major(dai_module=None) -> int:
    """Detect DepthAI 2/3 from the Pipeline class without instantiating it."""
    module = dai if dai_module is None else dai_module
    if module is None:
        raise RuntimeError("stack_lane OAK mode requires depthai")
    pipeline_class = getattr(module, "Pipeline", None)
    if pipeline_class is None:
        raise RuntimeError("depthai.Pipeline is unavailable")
    return 3 if callable(getattr(pipeline_class, "start", None)) else 2


def validate_depthai_version(dai_module=None) -> tuple[int, tuple[int, ...]]:
    """Require a supported runtime matching the feature-detected API major."""
    module = dai if dai_module is None else dai_module
    api_major = depthai_api_major(module)
    raw_version = getattr(module, "__version__", "")
    if not isinstance(raw_version, str):
        raw_version = ""
    numbers = tuple(int(value) for value in re.findall(r"\d+", raw_version))
    minimum = MINIMUM_DEPTHAI_VERSION.get(api_major)
    version_major = numbers[0] if numbers else None
    if (
        minimum is None
        or version_major != api_major
        or numbers[:len(minimum)] < minimum
    ):
        required = ".".join(str(value) for value in minimum or ())
        raise LaneOakConfigurationError(
            "unsupported DepthAI runtime: "
            f"detected={raw_version or 'unknown'}, API={api_major}.x, "
            f"required>={required}, runtime/API major must match"
        )
    return api_major, numbers


def estimate_rgb_payload_bytes_per_sec(
    width: int,
    height: int,
    fps: float,
) -> float:
    """Estimate the uncompressed BGR payload sent over USB."""
    return float(width * height) * float(fps) * 3.0


def validate_camera_profile(
    *,
    width: int,
    height: int,
    fps: float,
    usb_speed: str,
) -> None:
    """Reject invalid or unsafe USB2 profiles before opening hardware."""
    if width <= 0 or height <= 0 or fps <= 0.0:
        raise ValueError("camera width, height and fps must be positive")
    normalized_speed = normalize_usb_speed(usb_speed)
    if normalized_speed != "high":
        return
    payload = estimate_rgb_payload_bytes_per_sec(width, height, fps)
    if payload <= USB2_SAFE_PAYLOAD_BYTES_PER_SEC:
        return
    raise ValueError(
        "USB2(HIGH) safe payload exceeded: "
        f"{width}x{height}@{fps:g}, "
        f"payload={payload / 1_000_000:.1f}MB/s. "
        "Use the production 1280x720@10 profile."
    )


def _get_device_id(device_or_info) -> str:
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


def _usb_speed_enum(dai_module, requested_usb_speed: str):
    normalized = normalize_usb_speed(requested_usb_speed)
    try:
        return getattr(dai_module.UsbSpeed, normalized.upper())
    except AttributeError as error:
        raise LaneOakConfigurationError(
            f"DepthAI does not expose UsbSpeed.{normalized.upper()}"
        ) from error


def verify_opened_device(
    device,
    *,
    requested_mxid: str,
    requested_usb_speed: str,
) -> tuple[str, str]:
    """Fail closed on an unexpected camera or non-HIGH vehicle link."""
    actual_mxid = _get_device_id(device)
    if actual_mxid == "unknown":
        raise LaneOakConfigurationError(
            "opened OAK did not report a usable device ID"
        )
    if requested_mxid and actual_mxid != requested_mxid:
        raise LaneOakConfigurationError(
            "opened OAK does not match camera_mxid: "
            f"requested={requested_mxid}, actual={actual_mxid}"
        )
    actual_usb_speed = _get_usb_speed_name(device)
    if actual_usb_speed == "unknown":
        raise LaneOakConfigurationError(
            "opened OAK did not report its actual USB speed"
        )
    if requested_usb_speed == "high" and actual_usb_speed != "high":
        raise LaneOakConfigurationError(
            "USB2(HIGH) was requested but the actual link is not HIGH: "
            f"actual={actual_usb_speed}"
        )
    return actual_mxid, actual_usb_speed


def _connected_device_infos(dai_module):
    getter = getattr(dai_module.Device, "getAllConnectedDevices", None)
    if getter is None:
        getter = getattr(dai_module.Device, "getAllAvailableDevices", None)
    if getter is None:
        raise RuntimeError("DepthAI device enumeration API is unavailable")
    return list(getter())


def _select_device_info(dai_module, requested_mxid: str):
    if requested_mxid:
        return dai_module.DeviceInfo(requested_mxid)
    device_infos = _connected_device_infos(dai_module)
    if not device_infos:
        raise RuntimeError("no OAK device is available")
    if len(device_infos) > 1:
        connected = [_get_device_id(info) for info in device_infos]
        raise LaneOakConfigurationError(
            "camera_mxid is required when multiple OAK devices are present: "
            f"connected={connected}"
        )
    return device_infos[0]


def _close_device_noexcept(device) -> None:
    try:
        device.close()
    except (AttributeError, RuntimeError):
        pass


def _open_with_retry(
    *,
    dai_module,
    requested_mxid: str,
    opener: Callable[[object], object],
    warn: Optional[Callable[[str], None]],
    sleep_fn: Callable[[float], None],
    max_attempts: int,
):
    last_error: Optional[Exception] = None
    for attempt in range(1, max_attempts + 1):
        try:
            device_info = _select_device_info(
                dai_module,
                requested_mxid,
            )
            return opener(device_info)
        except LaneOakConfigurationError:
            raise
        except (RuntimeError, OSError) as error:
            last_error = error
        if attempt < max_attempts:
            if warn is not None:
                warn(
                    "OAK open failed "
                    f"({attempt}/{max_attempts}): {last_error}; "
                    f"retrying in {OPEN_RETRY_INTERVAL_SEC:g}s"
                )
            sleep_fn(OPEN_RETRY_INTERVAL_SEC)
    target = requested_mxid or "single connected OAK"
    raise RuntimeError(
        f"failed to open {target} after {max_attempts} attempts: "
        f"{last_error}"
    ) from last_error


def open_oak_v2_device(
    pipeline,
    *,
    mxid: str,
    usb_speed: str,
    dai_module=None,
    warn: Optional[Callable[[str], None]] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    max_attempts: int = OPEN_MAX_ATTEMPTS,
):
    """Open a DepthAI 2 device with explicit DeviceInfo and UsbSpeed."""
    module = dai if dai_module is None else dai_module
    if module is None:
        raise RuntimeError("stack_lane OAK mode requires depthai")
    requested_mxid = str(mxid).strip()
    requested_usb_speed = normalize_usb_speed(usb_speed)
    speed_enum = _usb_speed_enum(module, requested_usb_speed)

    def opener(device_info):
        selected_mxid = requested_mxid or _get_device_id(device_info)
        if selected_mxid == "unknown":
            raise LaneOakConfigurationError(
                "selected DeviceInfo did not report a usable device ID"
            )
        device = module.Device(pipeline, device_info, speed_enum)
        try:
            verify_opened_device(
                device,
                requested_mxid=selected_mxid,
                requested_usb_speed=requested_usb_speed,
            )
        except Exception:
            _close_device_noexcept(device)
            raise
        return device

    return _open_with_retry(
        dai_module=module,
        requested_mxid=requested_mxid,
        opener=opener,
        warn=warn,
        sleep_fn=sleep_fn,
        max_attempts=max_attempts,
    )


def open_oak_v3_device(
    *,
    mxid: str,
    usb_speed: str,
    dai_module=None,
    warn: Optional[Callable[[str], None]] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    max_attempts: int = OPEN_MAX_ATTEMPTS,
):
    """Open a DepthAI 3 device before constructing its Pipeline."""
    module = dai if dai_module is None else dai_module
    if module is None:
        raise RuntimeError("stack_lane OAK mode requires depthai")
    requested_mxid = str(mxid).strip()
    requested_usb_speed = normalize_usb_speed(usb_speed)
    speed_enum = _usb_speed_enum(module, requested_usb_speed)

    def opener(device_info):
        selected_mxid = requested_mxid or _get_device_id(device_info)
        if selected_mxid == "unknown":
            raise LaneOakConfigurationError(
                "selected DeviceInfo did not report a usable device ID"
            )
        device = module.Device(device_info, speed_enum)
        try:
            verify_opened_device(
                device,
                requested_mxid=selected_mxid,
                requested_usb_speed=requested_usb_speed,
            )
        except Exception:
            _close_device_noexcept(device)
            raise
        return device

    return _open_with_retry(
        dai_module=module,
        requested_mxid=requested_mxid,
        opener=opener,
        warn=warn,
        sleep_fn=sleep_fn,
        max_attempts=max_attempts,
    )


class OakRgbCamera:
    """DepthAI 2/3 RGB camera with deterministic device selection."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: float,
        mxid: str,
        usb_speed: str,
        warn: Optional[Callable[[str], None]] = None,
    ) -> None:
        """Open the selected OAK RGB camera with a validated runtime."""
        module = dai
        if module is None:
            raise RuntimeError("stack_lane OAK mode requires depthai")
        self.requested_mxid = str(mxid).strip()
        self.requested_usb_speed = normalize_usb_speed(usb_speed)
        validate_camera_profile(
            width=width,
            height=height,
            fps=fps,
            usb_speed=self.requested_usb_speed,
        )
        session = open_oak_session(
            width=width,
            height=height,
            fps=fps,
            mxid=self.requested_mxid,
            usb_speed=self.requested_usb_speed,
            dai_module=module,
            warn=warn,
        )
        self.api_major = session.api_major
        self.pipeline = session.pipeline
        self.device = session.device
        self.queue = session.queue
        self.mxid = _get_device_id(session.device)
        self.usb_speed = _get_usb_speed_name(session.device).upper()

    def release(self) -> None:
        """Close the queue, pipeline and device exactly once."""
        queue = self.queue
        pipeline = self.pipeline
        device = self.device
        self.queue = None
        self.pipeline = None
        self.device = None
        _close_session_noexcept(
            _OakSession(self.api_major, pipeline, device, queue)
        )


def open_oak_session(
    *,
    width: int,
    height: int,
    fps: float,
    mxid: str,
    usb_speed: str,
    dai_module=None,
    warn: Optional[Callable[[str], None]] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    max_attempts: int = OPEN_MAX_ATTEMPTS,
) -> _OakSession:
    """Create the complete device, pipeline and queue as one retry unit."""
    module = dai if dai_module is None else dai_module
    if module is None:
        raise RuntimeError("stack_lane OAK mode requires depthai")
    requested_mxid = str(mxid).strip()
    requested_usb_speed = normalize_usb_speed(usb_speed)
    api_major, _depthai_version = validate_depthai_version(module)
    last_error: Optional[Exception] = None

    for attempt in range(1, max_attempts + 1):
        try:
            if api_major == 2:
                return _create_v2_session_once(
                    module,
                    width=width,
                    height=height,
                    fps=fps,
                    mxid=requested_mxid,
                    usb_speed=requested_usb_speed,
                )
            return _create_v3_session_once(
                module,
                width=width,
                height=height,
                fps=fps,
                mxid=requested_mxid,
                usb_speed=requested_usb_speed,
            )
        except LaneOakConfigurationError:
            raise
        except (RuntimeError, OSError) as error:
            last_error = error
        if attempt < max_attempts:
            if warn is not None:
                warn(
                    "OAK session setup failed "
                    f"({attempt}/{max_attempts}): {last_error}; "
                    f"retrying in {OPEN_RETRY_INTERVAL_SEC:g}s"
                )
            sleep_fn(OPEN_RETRY_INTERVAL_SEC)

    target = requested_mxid or "single connected OAK"
    raise RuntimeError(
        f"failed to create the complete OAK session for {target} "
        f"after {max_attempts} attempts: {last_error}"
    ) from last_error


def _create_v2_session_once(
    dai_module,
    *,
    width: int,
    height: int,
    fps: float,
    mxid: str,
    usb_speed: str,
) -> _OakSession:
    pipeline = _build_v2_pipeline(dai_module, width, height, fps)
    device = None
    queue = None
    try:
        device = open_oak_v2_device(
            pipeline,
            mxid=mxid,
            usb_speed=usb_speed,
            dai_module=dai_module,
            max_attempts=1,
        )
        queue = device.getOutputQueue(
            "rgb",
            maxSize=4,
            blocking=False,
        )
        return _OakSession(2, pipeline, device, queue)
    except Exception:
        _close_session_noexcept(_OakSession(2, pipeline, device, queue))
        raise


def _create_v3_session_once(
    dai_module,
    *,
    width: int,
    height: int,
    fps: float,
    mxid: str,
    usb_speed: str,
) -> _OakSession:
    device = None
    pipeline = None
    queue = None
    try:
        device = open_oak_v3_device(
            mxid=mxid,
            usb_speed=usb_speed,
            dai_module=dai_module,
            max_attempts=1,
        )
        pipeline = dai_module.Pipeline(device)
        camera = pipeline.create(dai_module.node.Camera).build(
            dai_module.CameraBoardSocket.CAM_A
        )
        queue = camera.requestOutput(
            size=(width, height),
            type=dai_module.ImgFrame.Type.BGR888i,
            fps=float(fps),
        ).createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()
        return _OakSession(3, pipeline, device, queue)
    except Exception:
        _close_session_noexcept(_OakSession(3, pipeline, device, queue))
        raise


def _build_v2_pipeline(dai_module, width: int, height: int, fps: float):
    pipeline = dai_module.Pipeline()
    camera = pipeline.createColorCamera()
    camera.setPreviewSize(width, height)
    camera.setInterleaved(False)
    camera.setBoardSocket(dai_module.CameraBoardSocket.CAM_A)
    camera.setFps(fps)
    output = pipeline.createXLinkOut()
    output.setStreamName("rgb")
    output.input.setBlocking(False)
    output.input.setQueueSize(1)
    camera.preview.link(output.input)
    return pipeline


def _close_queue_noexcept(queue) -> None:
    if queue is None:
        return
    try:
        queue.close()
    except (AttributeError, RuntimeError):
        pass


def _close_session_noexcept(session: _OakSession) -> None:
    _close_queue_noexcept(session.queue)
    if session.api_major >= 3:
        _stop_pipeline_noexcept(session.pipeline)
    if session.device is not None:
        _close_device_noexcept(session.device)


def _stop_pipeline_noexcept(pipeline) -> None:
    if pipeline is None:
        return
    try:
        pipeline.stop()
    except (AttributeError, RuntimeError):
        pass
    try:
        pipeline.wait()
    except (AttributeError, RuntimeError):
        pass
