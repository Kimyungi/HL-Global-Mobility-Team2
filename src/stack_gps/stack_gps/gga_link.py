"""로버 시리얼 링크 — NMEA GGA 수신 + 베이스 RTCM 주입 (백그라운드 스레드).

record_waypoints.py와 같은 구조·같은 이유: FST-UEF9P의 시리얼 포트는 한
프로세스만 열어야 하므로(문장 조각남), GGA를 읽는 이 노드가 RTCM 주입까지
겸한다. rtcm_host를 빈 문자열로 주면 주입은 끄고 수신만 한다
(예: SiK 텔레메트리로 RTCM이 다른 경로로 들어올 때).

스레드는 최신 fix 하나만 공유 상태로 유지하고, 노드 타이머가 pull한다
(CLAUDE.md §5.2의 스냅샷 원칙과 동일한 모양).
"""
import math
import socket
import threading
import time

import serial


def parse_gga(line):
    """GGA 문장 → (utc, lat, lon, h_ellip, quality) 또는 None. (record_waypoints와 동일)"""
    f = line.split(",")
    if len(f) < 12 or not f[2] or not f[4] or not f[6].isdigit():
        return None
    try:
        lat = int(f[2][:2]) + float(f[2][2:]) / 60.0
        if f[3] == "S":
            lat = -lat
        lon = int(f[4][:3]) + float(f[4][3:]) / 60.0
        if f[5] == "W":
            lon = -lon
        alt_msl = float(f[9]) if f[9] else 0.0
        geoid = float(f[11]) if f[11] else 0.0
        return f[1], lat, lon, alt_msl + geoid, int(f[6])
    except ValueError:
        return None


class GgaLink:
    def __init__(self, serial_port, baud=115200, rtcm_host="", rtcm_port=2101,
                 log=print):
        self._serial_port = serial_port
        self._baud = baud
        self._rtcm_host = rtcm_host
        self._rtcm_port = rtcm_port
        self._log = log
        self._lock = threading.Lock()
        self._fix = None          # (lat, lon, h, quality, monotonic_t)
        self._rtcm_bytes = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def latest_fix(self):
        """(lat, lon, h, quality, age_s) 또는 None — 스레드 안전 스냅샷."""
        with self._lock:
            if self._fix is None:
                return None
            lat, lon, h, q, t = self._fix
            return lat, lon, h, q, time.monotonic() - t

    def rtcm_rate_and_reset(self):
        with self._lock:
            b, self._rtcm_bytes = self._rtcm_bytes, 0
        return b

    def _connect_rtcm(self):
        if not self._rtcm_host:
            return None
        s = socket.create_connection((self._rtcm_host, self._rtcm_port), timeout=5)
        s.setblocking(False)
        return s

    def _run(self):
        ser, sock, nmea_buf = None, None, b""
        while not self._stop.is_set():
            try:
                if ser is None:
                    ser = serial.Serial(self._serial_port, self._baud, timeout=0.2)
                    self._log(f"로버 시리얼 연결: {self._serial_port}")
                if sock is None and self._rtcm_host:
                    sock = self._connect_rtcm()
                    self._log(f"베이스 RTCM 연결: {self._rtcm_host}:{self._rtcm_port}")

                # ① RTCM 펌프 (논블로킹 — 밀린 만큼 전부 로버로)
                if sock is not None:
                    try:
                        while True:
                            d = sock.recv(4096)
                            if not d:
                                raise ConnectionError("RTCM 서버 연결 종료")
                            ser.write(d)
                            with self._lock:
                                self._rtcm_bytes += len(d)
                    except (BlockingIOError, InterruptedError):
                        pass
                    except OSError as e:
                        self._log(f"⚠ RTCM 연결 오류: {e} — 재접속 예정")
                        try:
                            sock.close()
                        except OSError:
                            pass
                        sock = None

                # ② NMEA 수신·GGA 파싱
                nmea_buf += ser.read(ser.in_waiting or 1)
                while b"\n" in nmea_buf:
                    raw, nmea_buf = nmea_buf.split(b"\n", 1)
                    line = raw.decode(errors="ignore").strip()
                    if not (line.startswith("$G") and "GGA" in line[:7]):
                        continue
                    parsed = parse_gga(line)
                    if parsed is None:
                        continue
                    _, lat, lon, h, quality = parsed
                    if not (math.isfinite(lat) and math.isfinite(lon)):
                        continue
                    with self._lock:
                        self._fix = (lat, lon, h, quality, time.monotonic())

            except (OSError, serial.SerialException) as e:
                self._log(f"⚠ 시리얼/네트워크 오류: {e} — 3초 후 재시도")
                for res in (ser, sock):
                    try:
                        if res is not None:
                            res.close()
                    except OSError:
                        pass
                ser, sock, nmea_buf = None, None, b""
                self._stop.wait(3.0)

        for res in (ser, sock):
            try:
                if res is not None:
                    res.close()
            except OSError:
                pass
