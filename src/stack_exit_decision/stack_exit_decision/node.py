"""Perception only. Inference/camera failures produce no votes; MGM owns timeout.

The worker keeps slow model/camera operations out of the ROS control callback.
No CAN, stop requests or route commands are published by this process.
"""
import math
from pathlib import Path
import threading
import time

from .last_mission_state import detections_from_result, validate_model_classes


def best_detection(detections):
    valid = [d for d in detections if d.class_id in (0, 1) and
             math.isfinite(d.confidence) and .5 <= d.confidence <= 1.]
    if not valid:
        return -1, 0.
    confidence = max(d.confidence for d in valid)
    classes = {d.class_id for d in valid if d.confidence == confidence}
    return (classes.pop(), confidence) if len(classes) == 1 else (-1, 0.)


def detector_requested(status, received_ns, now_ns):
    return (status is not None and 0 <= now_ns - received_ns <= 250_000_000
            and status.revised_v2 and status.go_authorized and not status.estop_active
            and status.last_mission_phase in (status.LAST_APPROACH, status.LAST_STOPPING, status.LAST_JUDGING))


def main(args=None):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from fma_interfaces.msg import ExitDetection, MgmState
    from sensor_msgs.msg import Image

    class ExitDetector(Node):
        def __init__(self):
            super().__init__('exit_detector')
            default = Path(get_package_share_directory('stack_exit_decision')) / 'models/exit_decision_yolo26n.pt'
            self.model_path = self.declare_parameter('model_path', str(default)).value
            self.image_topic = self.declare_parameter('image_topic', '').value
            self.mxid = self.declare_parameter('oak_mxid', '14442C10B167CFD200').value
            self.usb_speed = self.declare_parameter('oak_usb_speed', 'high').value
            self.exposure = self.declare_parameter('oak_exposure_compensation', 4).value
            self.device_name = self.declare_parameter('device', 'cpu').value
            self.lock = threading.Lock()
            self.status, self.received_ns, self.frame = None, 0, None
            self.stopping = threading.Event()
            self.pub = self.create_publisher(ExitDetection, '/perception/exit_detection', 10)
            self.create_subscription(MgmState, '/adas/mgm_state', self.control, 1)
            if self.image_topic:
                self.create_subscription(Image, self.image_topic, self.image, qos_profile_sensor_data)
            self.worker = threading.Thread(target=self.run_worker, daemon=True)
            self.worker.start()

        def control(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age = self.get_clock().now().nanoseconds - stamp
            with self.lock:
                self.status = msg if stamp > 0 and 0 <= age <= 250_000_000 else None
                self.received_ns = time.monotonic_ns()

        def image(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if msg.encoding not in ('bgr8', 'rgb8') or msg.step < msg.width * 3 or not msg.width or not msg.height:
                return
            if len(msg.data) != msg.step * msg.height:
                return
            pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = pixels[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = frame[:, :, ::-1]
            with self.lock:
                self.frame = stamp, frame.copy()

        def run_worker(self):
            camera = None
            try:
                # Same bounded CPU runtime as the traffic model.
                from stack_traffic import omp_runtime  # noqa: F401
                from ultralytics import YOLO
                model = YOLO(self.model_path)
                validate_model_classes(model.names)
                if model.task != 'detect':
                    raise ValueError('exit model must be an object detector')
                last_stamp = 0
                while not self.stopping.wait(.02):
                    with self.lock:
                        status, received, image = self.status, self.received_ns, self.frame
                    if not detector_requested(status, received, time.monotonic_ns()):
                        if camera is not None:
                            camera.release()
                            camera = None
                        continue
                    try:
                        if not self.image_topic:
                            from stack_traffic.oak_camera import OakRgbdCamera, dai
                            if camera is None:
                                camera = OakRgbdCamera(
                                    width=640, height=480, fps=10., depth_enabled=False,
                                    depth_confidence_threshold=200, depth_left_right_check=False,
                                    depth_subpixel=False, depth_median_filter_size=0,
                                    depth_decimation_factor=1, depth_speckle_filter=False,
                                    depth_spatial_filter=False, depth_temporal_filter=False,
                                    minimum_depth_m=.1, maximum_depth_m=20., mxid=self.mxid,
                                    usb_speed=self.usb_speed, exposure_compensation=self.exposure)
                            packet = camera.queue.tryGet()
                            if packet is None:
                                continue
                            age = (dai.Clock.now() - packet.getTimestamp()).total_seconds()
                            if not 0 <= age <= .5:
                                continue
                            stamp = self.get_clock().now().nanoseconds - int(age * 1e9)
                            image = stamp, packet.getCvFrame()
                        with self.lock:
                            status, received = self.status, self.received_ns
                        if (image is None or not detector_requested(status, received, time.monotonic_ns())
                                or status.last_mission_phase not in (status.LAST_APPROACH, status.LAST_JUDGING)
                                or not status.last_mission_detector_enabled):
                            continue
                        stamp, frame = image
                        age_ns = self.get_clock().now().nanoseconds - stamp
                        if stamp <= last_stamp or not 0 <= age_ns <= 500_000_000:
                            continue
                        last_stamp = stamp
                        result = model.predict(frame, imgsz=640, conf=.5, device=self.device_name, verbose=False)[0]
                        cls, confidence = best_detection(detections_from_result(result))
                        msg = ExitDetection(request_id=status.last_mission_request_id,
                                            class_id=cls, confidence=float(confidence))
                        msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(stamp, 1_000_000_000)
                        self.pub.publish(msg)
                    except Exception as error:
                        self.get_logger().error(f'Exit observation unavailable: {error}')
                        if camera is not None:
                            camera.release()
                            camera = None
                        self.stopping.wait(.5)
            except Exception as error:
                self.get_logger().error(f'Exit model unavailable; MGM will use its 3-second fallback: {error}')
            finally:
                if camera is not None:
                    camera.release()

        def close(self):
            self.stopping.set()
            self.worker.join(timeout=3.)

    rclpy.init(args=args)
    node = ExitDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
