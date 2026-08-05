"""HandsFree IMU 시리얼 링크 — 오일러각·관성 프레임 수신 (백그라운드 스레드).

장치: HandsFree IMU (HFI 계열, CP2102 브리지, /dev/ttyUSB_IMU, 921600 baud).
프레임 (2026-08-01 실기 캡처로 확인, CRC까지 검증):

  [0]=0xAA [1]=0x55 [2]=type [3..6]=길이·예약 [7:11]=u32 타임스탬프
  [11:-2]=f32 페이로드(LE) [-2:]=CRC-16/MODBUS(LE, bytes[2:-2] 대상)

  type 0x14 (25B): roll, pitch, yaw [deg] — AHRS 오일러각
  type 0x2C (49B): gyro xyz [rad/s], accel xyz [g], mag xyz

GgaLink와 같은 구조: 스레드가 최신 샘플 하나만 공유 상태로 유지하고
노드 타이머가 pull 한다. 파서는 순수 함수로 분리 — 시리얼 없이 테스트 가능.
"""
import math
import struct
import threading
import time

import serial

_FRAME_LEN = {0x14: 25, 0x2C: 49}


def crc16_modbus(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def parse_stream(buf):
    """수신 버퍼 → (frames, 남은 버퍼, crc_err 수).

    frames: [('euler', roll, pitch, yaw)] (rad) 또는
            [('inertial', gx, gy, gz, ax, ay, az)] (rad/s, g).
    불완전 프레임은 버퍼에 남기고, 헤더/CRC 불일치는 1바이트 전진으로 재동기.
    """
    frames, crc_err, i = [], 0, 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0:
            return frames, buf[max(len(buf) - 1, i):], crc_err
        if j + 3 > len(buf):
            return frames, buf[j:], crc_err
        ftype = buf[j + 2]
        flen = _FRAME_LEN.get(ftype)
        if flen is None:
            i = j + 2
            continue
        if j + flen > len(buf):
            return frames, buf[j:], crc_err
        frame = buf[j:j + flen]
        if crc16_modbus(frame[2:-2]) != struct.unpack("<H", frame[-2:])[0]:
            crc_err += 1
            i = j + 2
            continue
        if ftype == 0x14:
            r, p, y = struct.unpack("<3f", frame[11:23])
            frames.append(('euler', math.radians(r), math.radians(p),
                           math.radians(y)))
        else:
            frames.append(('inertial', *struct.unpack("<6f", frame[11:35])))
        i = j + flen


class ImuLink:
    def __init__(self, serial_port, baud=921600, log=print):
        self._serial_port = serial_port
        self._baud = baud
        self._log = log
        self._lock = threading.Lock()
        self._euler = None    # (roll, pitch, yaw[rad], monotonic_t)
        self._gyro_z = None   # (wz[rad/s], monotonic_t)
        self._frames = 0
        self._crc_err = 0
        self._gen = 0   # 시리얼 (재)연결 세대 — 재연결 = 전원 재인가 가능성
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def latest_euler(self):
        """(roll, pitch, yaw[rad], age_s) 또는 None — 스레드 안전 스냅샷."""
        with self._lock:
            if self._euler is None:
                return None
            r, p, y, t = self._euler
            return r, p, y, time.monotonic() - t

    def latest_gyro_z(self):
        """(wz[rad/s], age_s) 또는 None."""
        with self._lock:
            if self._gyro_z is None:
                return None
            wz, t = self._gyro_z
            return wz, time.monotonic() - t

    def stats_and_reset(self):
        """(정상 프레임 수, CRC 오류 수) — 상태 로그용, 호출 시 리셋."""
        with self._lock:
            out = (self._frames, self._crc_err)
            self._frames = self._crc_err = 0
        return out

    def generation(self):
        """시리얼 (재)연결 횟수. 값이 바뀌면 USB 재삽입 등으로 장치 전원이
        재인가됐을 수 있다 → IMU yaw 기준점 리셋 가능성 (융합 오프셋 무효)."""
        with self._lock:
            return self._gen

    def _run(self):
        ser, buf = None, b""
        while not self._stop.is_set():
            try:
                if ser is None:
                    ser = serial.Serial(self._serial_port, self._baud,
                                        timeout=0.2)
                    with self._lock:
                        self._gen += 1
                    self._log(f"IMU 시리얼 연결: {self._serial_port}")
                buf += ser.read(ser.in_waiting or 1)
                frames, buf, crc_err = parse_stream(buf)
                if len(buf) > 4096:   # 재동기 실패가 누적되면 버림
                    buf = b""
                if not frames and not crc_err:
                    continue
                now = time.monotonic()
                with self._lock:
                    self._crc_err += crc_err
                    self._frames += len(frames)
                    for f in frames:
                        if f[0] == 'euler':
                            self._euler = (f[1], f[2], f[3], now)
                        else:
                            self._gyro_z = (f[3], now)
            except (OSError, serial.SerialException) as e:
                self._log(f"⚠ IMU 시리얼 오류: {e} — 3초 후 재시도")
                try:
                    if ser is not None:
                        ser.close()
                except OSError:
                    pass
                ser, buf = None, b""
                self._stop.wait(3.0)
        try:
            if ser is not None:
                ser.close()
        except OSError:
            pass
