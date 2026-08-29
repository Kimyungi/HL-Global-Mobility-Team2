#!/usr/bin/env python3
"""전 센서 일괄 점검 — 확정된 USB 배치가 그대로인지 한 번에 본다.  (2026-08-29)

왜 필요한가
  센서가 7종(라이다 4 · 카메라 2 · IMU · GPS · CAN)이고, 고장이 대개
  **증상만 봐서는 원인을 못 찾는** 형태로 온다. 특히:
    · RPLiDAR 가 health OK 인데 /scan 0Hz  -> 같은 허브의 전류 부족 (HANDOVER §3.7)
    · OAK-D 가 USB3 로 열거          -> GPS C/N0 가 깎인다 (HANDOVER §3.1)
    · F9P 가 열거된 채 NMEA 만 죽음   -> USB 재열거 필요 (CLAUDE.md §6)
  그래서 "무엇이 살아 있나"가 아니라 **"확정된 배치와 같은가"**를 본다.

ROS 없이도 도는 항목(USB·심링크·IMU·GPS·CAN)과 ROS 가 필요한 항목(라이다 스캔)을
나눠서 검사한다. 라이다 스캔 검사는 드라이버가 떠 있을 때만 의미가 있다.

사용:
  python3 check_sensors.py            # 전체 (라이다는 드라이버가 떠 있어야 함)
  python3 check_sensors.py --no-ros   # USB·시리얼만 (드라이버 없이)
  python3 check_sensors.py --no-camera  # 카메라 부팅 생략 (GPS 측정 중일 때)
"""

import argparse
import glob
import os
import subprocess
import sys
import time

# 확정 배치 — 99-fma-lidars.rules 머리주석과 같은 원천이다.
EXPECT_LINKS = {
    'lidar_front': 'YD T-mini 전방',
    'lidar_rear': 'YD T-mini 후방',
    'lidar_left': 'RPLiDAR C1M1 좌',
    'lidar_right': 'RPLiDAR C1M1 우',
    'ttyUSB_IMU': 'HandsFree IMU',
    'ttyRover': 'u-blox F9P (GPS)',
    'ttyRadio': 'FTDI FT231X (RTK 라디오)',
}
EXPECT_MXID = {
    '14442C105157D3D200': 'stack_lane (차선)',
    '14442C10B167CFD200': 'stack_traffic (신호등·정지선)',
}
LIDAR_TOPICS = ('a1', 'a2', 'b1', 'b2')

OK, BAD, WARN = '  [OK]  ', '  [FAIL]', '  [WARN]'
_fails = []


def report(good, name, detail, warn=False):
    tag = OK if good else (WARN if warn else BAD)
    print(f'{tag} {name:<26} {detail}')
    if not good and not warn:
        _fails.append(name)


def check_links():
    print('\n== 고정 심링크 (udev) ==')
    for link, what in EXPECT_LINKS.items():
        path = '/dev/' + link
        if os.path.exists(path):
            report(True, link, f'-> {os.path.realpath(path)}  ({what})')
        else:
            report(False, link, f'없음 — udev 규칙 미설치? ({what})')


def check_hub_split():
    """카메라와 라이다의 허브 동거 여부 (HANDOVER §3.7).

    2026-08-29 에 **판정에서 경고로 내렸다.** 카메라에 전원·통신을 별도 라인으로
    빼면서 전류 경합의 원인 자체가 사라졌고, 앞으로는 허브 하나로 운용한다.
    그래도 지우지 않는 이유는 이 동거가 **고장처럼 안 보이는 증상**을 만들기
    때문이다 — RPLiDAR 가 `health OK` 인데 `/scan` 0Hz 면 여기부터 의심한다.
    """
    print('\n== 허브 배치 (카메라 ↔ 라이다) ==')

    def id_path(dev):
        try:
            out = subprocess.run(
                ['udevadm', 'info', '-q', 'property', '-n', dev],
                capture_output=True, text=True, timeout=5).stdout
        except (OSError, subprocess.SubprocessError):
            return None
        for line in out.splitlines():
            if line.startswith('ID_PATH='):
                return line.split('=', 1)[1]
        return None

    def root_hub(p):
        # pci-...-usb-0:3.2.4:1.0 -> '3.2'  (허브 갈래)
        try:
            chain = p.split('usb-0:')[1].split(':')[0]
        except (IndexError, AttributeError):
            return None
        return chain.split('.')[0]

    lidar_hubs = set()
    for link in ('lidar_front', 'lidar_rear', 'lidar_left', 'lidar_right'):
        p = id_path('/dev/' + link) if os.path.exists('/dev/' + link) else None
        h = root_hub(p)
        if h:
            lidar_hubs.add(h)
    if not lidar_hubs:
        report(False, '라이다 허브', '경로를 못 읽었다')
        return

    cam_hubs = set()
    for d in glob.glob('/sys/bus/usb/devices/*'):
        try:
            with open(os.path.join(d, 'idVendor')) as f:
                if f.read().strip() != '03e7':      # Movidius
                    continue
        except OSError:
            continue
        cam_hubs.add(os.path.basename(d).split('-')[1].split('.')[0])

    if not cam_hubs:
        report(True, '카메라 허브', '카메라 미연결 — 경합 없음', warn=True)
        return
    shared = lidar_hubs & cam_hubs
    if shared:
        report(False, '허브 배치',
               f'카메라와 라이다가 같은 허브({",".join(sorted(shared))}) — '
               '카메라 전원이 별도 라인일 때만 무해하다. RPLiDAR 가 health OK 인데 '
               '/scan 0Hz 면 여기부터 의심 (HANDOVER §3.7)', warn=True)
    else:
        report(True, '허브 배치',
               f'라이다 {sorted(lidar_hubs)} ↔ 카메라 {sorted(cam_hubs)} (분리)')


def check_imu():
    print('\n== IMU ==')
    path = '/dev/ttyUSB_IMU'
    if not os.path.exists(path):
        report(False, 'IMU', '심링크 없음')
        return
    try:
        import serial
    except ImportError:
        report(True, 'IMU', 'pyserial 없음 — 건너뜀', warn=True)
        return
    try:
        s = serial.Serial(path, 921600, timeout=1)
    except Exception as exc:                       # noqa: BLE001
        report(False, 'IMU', f'열기 실패: {exc}')
        return
    n, t0 = 0, time.time()
    while time.time() - t0 < 3:
        n += len(s.read(256))
    s.close()
    report(n > 100, 'IMU', f'3초간 {n} bytes @921600')


def check_gps():
    print('\n== GPS (u-blox F9P) ==')
    path = '/dev/ttyRover'
    if not os.path.exists(path):
        report(False, 'GPS', '심링크 없음')
        return
    try:
        import serial
    except ImportError:
        report(True, 'GPS', 'pyserial 없음 — 건너뜀', warn=True)
        return
    try:
        s = serial.Serial(path, 115200, timeout=1)
    except Exception as exc:                       # noqa: BLE001
        report(False, 'GPS', f'열기 실패: {exc}')
        return
    lines, t0 = [], time.time()
    while time.time() - t0 < 8:
        ln = s.readline().decode('ascii', 'replace').strip()
        if ln.startswith('$'):
            lines.append(ln)
    s.close()
    # NMEA 무수신 = "열거된 채 출력만 죽는" 고장 (CLAUDE.md §6)
    report(bool(lines), 'GPS NMEA',
           f'{len(lines)}줄/8초' if lines else '★ 무수신 — USB 재열거 필요')
    if not lines:
        return
    cn = []
    for ln in lines:
        if 'GSV' in ln[:6]:
            f = ln.split(',')
            for i in range(7, len(f) - 1, 4):
                try:
                    v = int(f[i])
                except (ValueError, IndexError):
                    continue
                if v > 0:
                    cn.append(v)
    if cn:
        cn.sort()
        top = cn[-8:]
        avg = sum(top) / len(top)
        # 실측 기준: USB2 39dB(FIXED) / USB3 22dB(DGPS 고착) — CLAUDE.md §6
        report(avg >= 35, 'GPS C/N0',
               f'상위8 평균 {avg:.1f} dB · 위성 {len(cn)}개'
               + ('' if avg >= 35 else '  (35 미만 = §3.1 USB3 간섭 의심)'),
               warn=25 <= avg < 35)
    else:
        report(True, 'GPS C/N0', '위성 0개 — 실내이거나 안테나 미연결', warn=True)
    for ln in reversed(lines):
        if 'GGA' in ln[:6]:
            f = ln.split(',')
            q = f[6] if len(f) > 6 else '?'
            name = {'0': '무측위', '1': '단독', '2': 'DGPS',
                    '4': 'RTK FIXED', '5': 'RTK FLOAT'}.get(q, q)
            report(q == '4', 'GPS fix', f'{q} ({name}) · 위성 {f[7]}',
                   warn=q != '4')
            break


def check_can():
    print('\n== CAN ==')
    try:
        out = subprocess.run(['ip', '-d', 'link', 'show', 'can0'],
                             capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError) as exc:
        report(False, 'can0', f'조회 실패: {exc}')
        return
    if out.returncode != 0:
        report(False, 'can0', '인터페이스 없음 — PCAN 미연결?')
        return
    up = 'state UP' in out.stdout or 'UP,LOWER_UP' in out.stdout
    rate = ''
    idx = out.stdout.find('bitrate ')
    if idx >= 0:
        rate = out.stdout[idx:].split()[1]
    report(up and rate == '1000000', 'can0',
           f'{"UP" if up else "DOWN"} · bitrate {rate or "?"}'
           + ('' if rate == '1000000' else '  (§1 권장 1 Mbps)'),
           warn=up and rate != '1000000')


def check_cameras():
    print('\n== 카메라 (OAK-D) ==')
    try:
        import depthai as dai
    except ImportError:
        report(True, '카메라', 'depthai 미설치 — 건너뜀', warn=True)
        return
    try:
        devs = dai.Device.getAllAvailableDevices()
    except Exception as exc:                       # noqa: BLE001
        report(False, '카메라 열거', str(exc)[:80])
        return
    found = {}
    for d in devs:
        mx = getattr(d, 'deviceId', None) or str(d.getDeviceId())
        found[mx] = d
    for mx, role in EXPECT_MXID.items():
        if mx not in found:
            report(False, f'OAK {role}', f'★ 미검출 (MxID {mx})')
            continue
        try:
            # ★ USB2 로 연다 — SuperSpeed 는 GNSS L1 을 덮는다 (HANDOVER §3.1)
            with dai.Pipeline(dai.Device(found[mx],
                                         maxUsbSpeed=dai.UsbSpeed.HIGH)) as p:
                dev = p.getDefaultDevice()
                cam = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                q = cam.requestOutput((640, 400), dai.ImgFrame.Type.NV12,
                                      fps=10).createOutputQueue()
                p.start()
                n, t0 = 0, time.time()
                while time.time() - t0 < 3:
                    if q.tryGet() is not None:
                        n += 1
                    time.sleep(0.01)
                report(n > 8, f'OAK {role}',
                       f'USB={dev.getUsbSpeed().name} · {n / 3:.1f} fps')
        except Exception as exc:                   # noqa: BLE001
            report(False, f'OAK {role}', f'{type(exc).__name__}: {str(exc)[:70]}')


def check_lidar_scans():
    print('\n== 라이다 스캔 (드라이버가 떠 있어야 함) ==')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ImportError:
        report(True, '라이다 스캔', 'ROS 2 환경 아님 — 건너뜀', warn=True)
        return

    class _C(Node):
        def __init__(self):
            super().__init__('check_sensors')
            self.n = dict.fromkeys(LIDAR_TOPICS, 0)
            for k in LIDAR_TOPICS:
                self.create_subscription(
                    LaserScan, f'/lidar/{k}/scan',
                    lambda _m, k=k: self.n.__setitem__(k, self.n[k] + 1),
                    qos_profile_sensor_data)

    rclpy.init()
    node = _C()
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    counts = dict(node.n)
    node.destroy_node()
    rclpy.shutdown()
    for k, v in counts.items():
        hz = v / 6.0
        report(hz > 5.0, f'/lidar/{k}/scan', f'{hz:.1f} Hz'
               + ('' if hz > 5.0 else '  ★ health OK 인데 0Hz 면 §3.7 전류 부족'))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--no-ros', action='store_true', help='라이다 스캔 검사 생략')
    ap.add_argument('--no-camera', action='store_true',
                    help='카메라 부팅 생략 (GPS 측정 중이면 권장)')
    args = ap.parse_args()

    print('전 센서 점검 — 확정 배치(2026-08-29)와 대조')
    check_links()
    check_hub_split()
    check_imu()
    check_gps()
    check_can()
    if not args.no_camera:
        check_cameras()
    if not args.no_ros:
        check_lidar_scans()

    print()
    if _fails:
        print(f'== 불합격 {len(_fails)}건: ' + ', '.join(_fails))
        return 1
    print('== 전 항목 통과 ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
