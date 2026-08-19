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

from stack_gps import usb_reset


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
        # sats/hdop: RTK 진단용 (2026-08-11 — DGPS 고착 원인 분리: 베이스 정상인데
        # FIXED 불가 시 로버 위성 가시성 확인 수단이 없었음)
        sats = int(f[7]) if f[7].isdigit() else 0
        hdop = float(f[8]) if f[8] else 0.0
        return f[1], lat, lon, alt_msl + geoid, int(f[6]), sats, hdop
    except ValueError:
        return None


KNOT_TO_MPS = 0.514444


def parse_rmc(line):
    """RMC 문장 → (speed_mps, yaw_enu_rad) 또는 None.

    course(진북 기준 시계방향 deg) → ENU yaw(동쪽 0, 반시계 +)로 변환.
    정지 상태에서는 course 필드가 비어 있으므로 None을 반환한다.
    """
    f = line.split(",")
    if len(f) < 9 or f[2] != "A" or not f[7] or not f[8]:
        return None
    try:
        speed = float(f[7]) * KNOT_TO_MPS
        yaw_enu = math.radians(90.0 - float(f[8]))
        return speed, yaw_enu
    except ValueError:
        return None


class GgaLink:
    def __init__(self, serial_port, baud=115200, rtcm_host="", rtcm_port=2101,
                 log=print, usb_reset_after_s=20.0, usb_reset_cooldown_s=60.0):
        self._serial_port = serial_port
        self._baud = baud
        self._rtcm_host = rtcm_host
        self._rtcm_port = rtcm_port
        self._log = log
        # ── NMEA 두절 → USB 리셋 복구 (2026-08-18 실차, usb_reset.py 주석 참조).
        # 판정은 **NMEA 한 줄도 안 오는 것**으로 한다. fix_quality 저하(DGPS/FLOAT)나
        # 위성 부족은 정상 범위의 전파 상황이고, 그걸로 리셋하면 멀쩡한 링크를
        # 끊어 되레 주행을 망친다. 실측 사망 사례는 "쓰기는 되는데 읽기가 0" 이었고,
        # 그것만이 사람이 뽑았다 꽂아야 했던 상태다.
        # 0 이하면 기능 끔.
        self._usb_reset_after_s = float(usb_reset_after_s)
        self._usb_reset_cooldown_s = float(usb_reset_cooldown_s)
        self._last_nmea_t = None      # 마지막으로 NMEA 를 **한 줄이라도** 받은 시각
        self._last_reset_t = None
        self._reset_count = 0
        self._lock = threading.Lock()
        self._fix = None          # (lat, lon, h, quality, monotonic_t)
        self._sat_info = (0, 0.0)  # (위성 수, HDOP) — 마지막 GGA 기준 (진단용)
        self._cog = None          # (speed_mps, yaw_enu, monotonic_t) — RMC 이동방향
        self._rtcm_bytes = 0
        self._stop = threading.Event()
        self._nmea_bytes = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def latest_sat_info(self):
        """(위성 수, HDOP) — 마지막 GGA 기준. RTK 품질 진단용."""
        with self._lock:
            return self._sat_info

    def latest_fix(self):
        """(lat, lon, h, quality, age_s, t_mono) 또는 None — 스레드 안전 스냅샷.

        t_mono는 수신 시각(monotonic) — 같은 값이면 같은 측정(새 GGA 아님).
        """
        with self._lock:
            if self._fix is None:
                return None
            lat, lon, h, q, t = self._fix
            return lat, lon, h, q, time.monotonic() - t, t

    def latest_cog(self):
        """(speed_mps, yaw_enu_rad, age_s) 또는 None — 이동방향(Course Over Ground).

        단일 안테나 GPS의 헤딩 근사 — 전진 주행 중에만 유효하다.
        속도·나이 판정은 호출자(노드)의 몫.
        """
        with self._lock:
            if self._cog is None:
                return None
            spd, yaw, t = self._cog
            return spd, yaw, time.monotonic() - t

    def rtcm_rate_and_reset(self):
        with self._lock:
            b, self._rtcm_bytes = self._rtcm_bytes, 0
        return b

    def nmea_rate_and_reset(self):
        """수신 NMEA 바이트 — RTCM 과 **짝으로** 봐야 고장이 구분된다.

        RTCM > 0 인데 NMEA == 0 이면 "쓰기는 되는데 수신기 출력이 죽은" 상태다
        (2026-08-18 실차의 그 고장). 둘 다 0 이면 시리얼 자체가 끊긴 것.
        """
        with self._lock:
            b, self._nmea_bytes = self._nmea_bytes, 0
        return b

    def usb_reset_count(self):
        return self._reset_count

    def _connect_rtcm(self):
        if not self._rtcm_host:
            return None
        s = socket.create_connection((self._rtcm_host, self._rtcm_port), timeout=5)
        s.setblocking(False)
        return s

    def _maybe_usb_reset(self, ser):
        """NMEA 가 usb_reset_after_s 동안 한 바이트도 없으면 USB 리셋. 실행했으면 True.

        수신기가 "열거는 됐는데 출력이 죽은" 상태를 푸는 유일한 수단이다
        (usb_reset.py 주석의 2026-08-18 실측). 포트를 **먼저 닫고** 리셋한다 —
        열린 채로 리셋하면 그 핸들이 무효가 되어 이후 read 가 EIO 로 떨어진다.
        쿨다운을 두는 이유: 리셋해도 안 살아나는 원인(안테나 탈락·수신기 전원)에서
        무한 리셋 루프에 빠지면 오히려 복구 기회를 없앤다.
        """
        if self._usb_reset_after_s <= 0.0:
            return False
        now = time.monotonic()
        if self._last_nmea_t is None:
            self._last_nmea_t = now          # 첫 연결 직후는 기준점만 잡는다
            return False
        if now - self._last_nmea_t < self._usb_reset_after_s:
            return False
        if (self._last_reset_t is not None and
                now - self._last_reset_t < self._usb_reset_cooldown_s):
            return False
        self._log(f"⚠ NMEA {now - self._last_nmea_t:.0f}초 두절 — USB 리셋 시도 "
                  f"(RTCM 쓰기는 되는데 읽기가 0이면 수신기가 걸린 것)")
        try:
            ser.close()
        except OSError:
            pass
        self._last_reset_t = now
        self._reset_count += 1
        if usb_reset.reset(self._serial_port, log=self._log):
            usb_reset.wait_for_tty(self._serial_port, log=self._log)
        self._last_nmea_t = time.monotonic()   # 재연결 후 다시 센다
        return True

    def _run(self):
        ser, sock, nmea_buf = None, None, b""
        next_rtcm_try = 0.0
        while not self._stop.is_set():
            try:
                if ser is None:
                    ser = serial.Serial(self._serial_port, self._baud, timeout=0.2)
                    self._log(f"로버 시리얼 연결: {self._serial_port}")
                # RTCM 연결 실패는 GGA 수신을 막지 않는다 (베이스 죽어도 위치는 계속)
                if sock is None and self._rtcm_host and time.monotonic() >= next_rtcm_try:
                    try:
                        sock = self._connect_rtcm()
                        self._log(f"베이스 RTCM 연결: {self._rtcm_host}:{self._rtcm_port}")
                    except OSError as e:
                        self._log(f"⚠ RTCM 연결 실패: {e} — 5초 후 재시도 (GGA 수신은 계속)")
                        next_rtcm_try = time.monotonic() + 5.0

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
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    with self._lock:
                        self._nmea_bytes += len(chunk)
                    self._last_nmea_t = time.monotonic()
                elif self._maybe_usb_reset(ser):
                    ser, sock, nmea_buf = None, None, b""
                    continue
                nmea_buf += chunk
                while b"\n" in nmea_buf:
                    raw, nmea_buf = nmea_buf.split(b"\n", 1)
                    line = raw.decode(errors="ignore").strip()
                    if not line.startswith("$G"):
                        continue
                    tag = line[3:6]
                    if tag == "GGA":
                        parsed = parse_gga(line)
                        if parsed is None:
                            continue
                        _, lat, lon, h, quality, sats, hdop = parsed
                        with self._lock:
                            self._sat_info = (sats, hdop)
                        if not (math.isfinite(lat) and math.isfinite(lon)):
                            continue
                        with self._lock:
                            self._fix = (lat, lon, h, quality, time.monotonic())
                    elif tag == "RMC":
                        rmc = parse_rmc(line)
                        if rmc is not None:
                            with self._lock:
                                self._cog = (rmc[0], rmc[1], time.monotonic())

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
