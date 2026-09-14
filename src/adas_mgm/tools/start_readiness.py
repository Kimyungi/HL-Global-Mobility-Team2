"""Start authorization only; runtime reference/E-stop/CAN gates remain in MGM."""


def fresh(stamp, now_ns, timeout=.5):
    ns = stamp.sec*1_000_000_000+stamp.nanosec
    age = (now_ns-ns)*1e-9
    return ns > 0 and 0 <= age <= timeout


def readiness(latest, now_ns, *, skip_gps=False, skip_camera=False, require_traffic=False):
    state = latest.get('state')
    current = state is not None and fresh(state.header.stamp, now_ns)
    camera = current and state.camera_available and not skip_camera
    gps = current and state.gps_fixed_ready and not skip_gps
    lidar = current and state.lidar_ready
    traffic = latest.get('traffic')
    traffic_ok = not require_traffic or (traffic is not None and fresh(traffic.header.stamp, now_ns))
    return bool(current and lidar and (camera or gps) and traffic_ok), bool(camera), bool(gps), bool(current), bool(traffic_ok)
