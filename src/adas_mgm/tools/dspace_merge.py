#!/usr/bin/env python3
"""dSPACE 로그 × rosbag 병합 분석 — "어디가 GPS/차선/회피였고, 명령 대비 실제는 어땠나".

dSPACE(하위)는 자기 시계로 MPC 출력 str·실제 str·수신 v_ref·실제 v를 찍고,
PC(상위)는 rosbag에 스테이트·ref·인지 상태를 찍는다. 두 로그는 **시계가 다르다**.
이 도구는 두 로그를 같은 시간축에 올린 뒤(§정렬), PC의 스테이트 구간으로 dSPACE
파형을 색칠해 "이 구간은 회피였다"를 바로 읽게 만든다.

정렬 방법 (자동 선택, 좋은 것부터):
  ① counter  — dSPACE가 TARGET_HEADER(0x100)의 counter를 로깅하면 **틱 단위 정확**.
               PC의 tick 열과 1:1 대응이라 시계 오차·드리프트가 원리적으로 없다.
  ② v_ref    — dSPACE가 수신한 목표속도 vs PC가 보낸 목표속도의 상호상관.
               rate limit 때문에 v_ref는 계단·램프가 많아 상관 피크가 뾰족하다.
               10ms 급 정렬 + 클럭 드리프트(전·후반 지연 차)까지 추정한다.
  ③ --lag    — 위 둘이 다 안 되면 사람이 직접 지연을 준다 (초).

사용:
  source ~/FMA_ws/install/setup.bash            # rosbag 역직렬화에 필요
  # 0) dSPACE CSV 열 이름 보기 (매핑 확인용)
  python3 dspace_merge.py --list-columns dspace_run1.csv
  # 1) 병합 분석
  python3 dspace_merge.py ~/FMA_ws/drive_logs/run_0816_203352 \
      --dspace dspace_run1.csv \
      --map "t=Time,v_cmd=v_ref_rx,v_act=v_meas,str_cmd=str_ref,str_act=str_meas"
  # 2) dSPACE 로그 없이 bag만 (스테이트·명령만 — 실제 str/v는 빈칸)
  python3 dspace_merge.py ~/FMA_ws/drive_logs/run_0816_203352
  # 3) 파이프라인 자체 점검 (bag에서 가짜 dSPACE CSV 생성 → 정렬이 복원되는지)
  python3 dspace_merge.py <run> --synth /tmp/fake_dspace.csv

출력: <run>/analysis_dspace/ 에 merged.csv + 그래프 4장 + report.txt
"""
import argparse
import csv
import math
import os
import re
import sqlite3
import sys
import warnings

import numpy as np

# 빈 구간의 nanmean·0 나눗셈은 정상 경로다 (해당 신호가 없는 스테이트 구간) → NaN 으로 둔다
warnings.filterwarnings("ignore", category=RuntimeWarning)
np.seterr(invalid="ignore", divide="ignore")

TICK = 0.01           # MGM 루프 주기 [s] — 병합 격자
L_WB = 0.595          # 휠베이스 [m] (조향각 ↔ 곡률 변환)
MAX_STEER_DEG = 27.3  # 조향 물리 한계
STATE_NAME = {0: "차선", 1: "GPS", 2: "회피", 3: "주차"}

# dataviz 검증 팔레트(light) — 슬롯 1~3 all-pairs 통과. 스테이트 색은 프로젝트 공용:
# 차선=파랑 / GPS=주황 / 회피=초록 (drive_logs 기존 분석 그림과 동일하게 유지할 것)
C_LANE, C_GPS, C_AVOID, C_PARK = "#2a78d6", "#eb6834", "#1baf7a", "#8a63d2"
C_CMD, C_ACT, C_GEOM = "#2a78d6", "#eb6834", "#a9a89f"
C_TRACK, C_CRIT, C_WARN = "#a9a89f", "#e34948", "#eda100"
INK, INK2, INK3 = "#0b0b0b", "#52514e", "#87857d"
SCOL = {0: C_LANE, 1: C_GPS, 2: C_AVOID, 3: C_PARK}
SBG = {0: "#eaf2fc", 1: "#fdeee7", 2: "#e6f6f0", 3: "#f0ebfa"}


# ─────────────────────────────────────────────────────────────── rosbag 읽기 ──
def read_bag(run_dir):
    """rosbag → 100Hz 틱 격자 딕셔너리. /adas/target_ref 가 격자의 기준."""
    from rclpy.serialization import deserialize_message
    from fma_interfaces.msg import TargetRef, VehicleVector, AvoidStatus, GpsPath, LanePath

    bag_dir = next((os.path.join(run_dir, d) for d in ("rosbag", "bag")
                    if os.path.isdir(os.path.join(run_dir, d))), None)
    if bag_dir is None:
        sys.exit(f"[bag] rosbag 디렉터리 없음: {run_dir}")
    db = [os.path.join(bag_dir, n) for n in sorted(os.listdir(bag_dir)) if n.endswith(".db3")][0]
    con = sqlite3.connect(db)
    tid = dict(con.execute("SELECT name, id FROM topics").fetchall())

    def msgs(topic, typ):
        if topic not in tid:
            return
        for ts, data in con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (tid[topic],)):
            yield ts * 1e-9, deserialize_message(bytes(data), typ)

    t, state, v_ref, npts = [], [], [], []
    x0, y0, yaw0, k0 = [], [], [], []
    for ts, m in msgs("/adas/target_ref", TargetRef):
        t.append(ts)
        state.append(m.state)
        v_ref.append(m.v_ref)
        npts.append(len(m.ref_points))
        p = m.ref_points[0] if m.ref_points else None
        x0.append(p.x if p else 0.0)
        y0.append(p.y if p else 0.0)
        yaw0.append(p.yaw if p else 0.0)
        k0.append(p.curvature if p else 0.0)
    if not t:
        sys.exit("[bag] /adas/target_ref 가 비어 있음 — MGM 출력이 없는 run")

    b = {k: np.asarray(v) for k, v in dict(
        t=t, state=state, v_ref=v_ref, n_pts=npts,
        x0=x0, y0=y0, yaw0=yaw0, k0=k0).items()}
    b["state"] = b["state"].astype(int)
    b["n_pts"] = b["n_pts"].astype(int)

    def hold(topic, typ, fields):
        """비동기 토픽 → 틱 격자에 zero-order hold (직전 값 유지)."""
        tt, rows = [], []
        for ts, m in msgs(topic, typ):
            tt.append(ts)
            rows.append([float(getattr(m, f)) for f in fields])
        out = {f: np.full(len(b["t"]), np.nan) for f in fields}
        if not tt:
            return out, 0
        tt = np.asarray(tt)
        rows = np.asarray(rows)
        idx = np.searchsorted(tt, b["t"], side="right") - 1
        ok = idx >= 0
        for j, f in enumerate(fields):
            out[f][ok] = rows[idx[ok], j]
        return out, len(tt)

    av, n_av = hold("/perception/avoid", AvoidStatus,
                    ["obstacle_detected", "avoidable", "ttc", "maneuver_done", "narrow_gap"])
    gp, n_gp = hold("/perception/gps_path", GpsPath,
                    ["cross_track_m", "fix_quality", "at_end", "heading_source"])
    ln, n_ln = hold("/perception/lane_path", LanePath, ["confidence"])
    b.update({f"avoid_{k}": v for k, v in av.items()})
    b.update({f"gps_{k}": v for k, v in gp.items()})
    b["lane_confidence"] = ln["confidence"]

    vv, n_vv = hold("/vehicle/vector", VehicleVector, ["v", "str", "x", "y", "yaw", "counter"])
    b.update({f"vv_{k}": v for k, v in vv.items()})

    b["_counts"] = dict(target_ref=len(b["t"]), avoid=n_av, gps_path=n_gp,
                        lane_path=n_ln, vehicle_vector=n_vv)
    return b


def read_lateral(run_dir):
    """lateral.csv (stack_gps, 100Hz epoch 타임스탬프) — 궤적 그림용."""
    p = os.path.join(run_dir, "lateral.csv")
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))
    if not rows:
        return None
    g = lambda k, d=0.0: np.array([float(r.get(k) or d) for r in rows])  # noqa: E731
    return dict(t=g("stamp_s"), lat=g("lat"), lon=g("lon"),
                cross=g("cross_track_m"), quality=g("quality"))


# ──────────────────────────────────────────────────────────── dSPACE 로그 ──
# ControlDesk/AutomationDesk CSV는 앞에 메타데이터 줄이 붙는 경우가 많다 →
# "숫자 행이 시작되는 지점"을 찾아 그 직전의 이름 줄을 헤더로 쓴다.
# 값: (부분일치 후보, 꼬리 정확일치 전용 후보). 뒤엣것은 "v"·"str"처럼 짧아서
# 아무 이름에나 걸리는 것들 — `servo_ma_filter.value` 가 v_act 로 잡히는 사고를 막는다.
ALIASES = {
    "t":       (["time", "sampletime", "timestamp", "abszeit"], ["t", "x"]),
    "counter": (["counter", "rxcounter", "hdrcounter", "pccounter"], []),
    "state":   (["state", "hdrstate", "drivestate", "mgmstate"], []),
    "n_pts":   (["npoints", "npts", "numpoints"], []),
    "v_cmd":   (["vref", "vcmd", "vtarget", "vdes", "targetspeed", "veltarget", "vsoll"], []),
    "v_act":   (["vact", "vmeas", "vreal", "vfb", "vehiclespeed", "vist", "motorma",
                 "wheelspeed", "encoderspeed"], ["v", "velocity", "speed"]),
    "str_cmd": (["strref", "strcmd", "steerref", "steercmd", "deltaref", "deltacmd",
                 "targetangle", "mpcstr", "strmpc", "steeringcmd", "cmdangle"], []),
    "str_act": (["stract", "strmeas", "strreal", "strfb", "steeract", "steeringangle",
                 "deltaact", "deltameas", "servoma", "servoangle"], ["str", "steer", "delta"]),
    "x0":      (["refx0", "refx"], ["x0"]),
    "y0":      (["refy0", "refy"], ["y0"]),
}


def _norm(s):
    return re.sub(r"[^a-z0-9]", "", s.lower())


def sniff_dspace(path):
    """구분자·헤더 줄·데이터 시작 줄을 찾아 (header, rows) 반환."""
    with open(path, newline="", errors="replace") as f:
        lines = f.read().splitlines()
    delim = max(",;\t", key=lambda d: sum(ln.count(d) for ln in lines[:80]))

    def flds(ln):
        return [c.strip().strip('"') for c in ln.split(delim)]

    def numeric_frac(ln):
        c = flds(ln)
        if len(c) < 2:
            return 0.0
        ok = 0
        for v in c:
            try:
                float(v.replace(",", ".") if delim != "," else v)
                ok += 1
            except ValueError:
                pass
        return ok / len(c)

    # ControlDesk는 데이터 시작 줄이 `trace_values,...` 로 명시된다. 일반 CSV는
    # "숫자 행 3줄 연속"으로 찾는다 — `trace_types,4,4,4,16,4,4` 같은 메타 줄이
    # 혼자서는 숫자 행처럼 보이기 때문 (실제로 이걸 데이터 시작으로 오인했다).
    start = next((i for i, ln in enumerate(lines) if ln.startswith("trace_values")), None)
    if start is None:
        start = next((i for i in range(len(lines) - 2)
                      if all(numeric_frac(lines[j]) > 0.8 for j in (i, i + 1, i + 2))
                      and len({len(flds(lines[j])) for j in (i, i + 1, i + 2)}) == 1), None)
    if start is None:
        sys.exit(f"[dspace] 숫자 데이터 행을 못 찾음: {path}")
    ncol = len(flds(lines[start]))
    # ControlDesk: 이름 줄이 `trace_names,,<신호>,...` (0열=행 라벨, 1열=시간, 빈 이름).
    # 데이터 바로 위 줄은 min/max 통계 줄이라 열 수만 보고 고르면 그걸 헤더로 잡는다.
    tn = next((i for i, ln in enumerate(lines) if ln.startswith("trace_names")), None)
    if tn is not None:
        header = flds(lines[tn])
        header[0] = "row_label"
        if len(header) > 1 and not header[1]:
            header[1] = "Time"
    else:
        hdr_i = next((i for i in range(start - 1, -1, -1) if len(flds(lines[i])) == ncol), None)
        header = flds(lines[hdr_i]) if hdr_i is not None else [f"col{i}" for i in range(ncol)]
    header = [h if h else f"col{i}" for i, h in enumerate(header)]

    def as_float(v):
        try:
            return float(v)
        except ValueError:
            return np.nan      # 행 라벨(`trace_values`)·빈 칸은 NaN 열로 흘려보낸다
    data = [[as_float(v) for v in flds(ln)] for ln in lines[start:] if len(flds(ln)) == ncol]
    return header, np.asarray(data, dtype=float)


def read_dspace(path, user_map=None, str_unit=None):
    header, data = sniff_dspace(path)
    if data.size == 0:
        sys.exit(f"[dspace] 데이터 행 0: {path}")
    idx_of = {}
    # ① 사용자 지정 매핑 우선 (열 이름 정확일치 → 부분일치 → 0-based 인덱스)
    for key, name in (user_map or {}).items():
        j = None
        if name in header:
            j = header.index(name)
        elif name.isdigit():
            j = int(name)
        else:
            cand = [i for i, h in enumerate(header) if _norm(name) in _norm(h)]
            j = cand[0] if cand else None
        if j is None:
            sys.exit(f"[dspace] --map 의 열을 못 찾음: {key}={name}\n  가진 열: {header}")
        idx_of[key] = j
    # ② 나머지는 이름으로 자동 추정 — ControlDesk 는 "블록경로/신호명" 이라 뒤쪽 조각으로 본다
    used = set(idx_of.values())
    for key, (loose, strict) in ALIASES.items():
        if key in idx_of:
            continue
        best = None
        for i, h in enumerate(header):
            if i in used:
                continue
            tail = _norm(h.split("/")[-1].split(".")[-1])
            whole = _norm(h)
            for rank, a in enumerate(loose):
                score = (0 if tail == a else 1 if tail.endswith(a) else
                         2 if a in whole else None)
                if score is not None and (best is None or (score, rank) < best[0]):
                    best = ((score, rank), i)
            for rank, a in enumerate(strict):     # 짧은 별명은 꼬리 정확일치만
                if tail == a and (best is None or (0, 100 + rank) < best[0]):
                    best = ((0, 100 + rank), i)
        if best:
            idx_of[key] = best[1]
            used.add(best[1])

    d = {k: data[:, j] for k, j in idx_of.items()}
    if "t" not in d:
        sys.exit(f"[dspace] 시간 열을 못 찾음 — --map 't=<열이름>' 지정 필요.\n  가진 열: {header}")
    # 시간 단위 추정 (ms/us 로 찍는 설정이 흔하다)
    span = float(np.nanmax(d["t"]) - np.nanmin(d["t"]))
    if span > 1e7:
        d["t"] *= 1e-6
    elif span > 1e4:
        d["t"] *= 1e-3
    # 조향 단위: rad 계약이지만 dSPACE 내부는 deg 인 경우가 많다
    for k in ("str_cmd", "str_act"):
        if k not in d:
            continue
        unit = str_unit or ("deg" if np.nanmax(np.abs(d[k])) > 1.6 else "rad")
        d[k + "_deg"] = d[k] if unit == "deg" else np.degrees(d[k])
    d["_header"] = header
    d["_map"] = {k: header[j] for k, j in idx_of.items()}
    return d


def harmonize_speed(bag, ds, forced=None):
    """dSPACE 속도 단위 판별 — dSPACE는 km/h로 찍는다 (CLAUDE.md §3). PC v_ref(m/s)와
    최대값 비가 3.6 부근이면 km/h로 보고 나눈다. 값이 바뀌므로 지문 대조 **전에** 한다."""
    if "v_cmd" not in ds:
        return "?"
    vb, vd = np.nanmax(np.abs(bag["v_ref"])), np.nanmax(np.abs(ds["v_cmd"]))
    r = vd / vb if vb > 0.05 else 1.0
    unit = forced or ("kmh" if 3.0 < r < 4.2 else "mps")
    if unit == "kmh":
        for k in ("v_cmd", "v_act"):
            if k in ds:
                ds[k] = ds[k] / 3.6
    return unit


# ─────────────────────────────────────────────────────────────────── 정렬 ──
def _fingerprint(state, v_ref, x0, y0):
    """한 틱의 지문 — dSPACE가 수신한 값과 PC가 보낸 값이 **양자화 후 정확히 같다**."""
    return (state.astype(np.int64) * 1_000_003
            + np.round(v_ref * 1000).astype(np.int64) * 1009
            + (np.round(x0 * 1000).astype(np.int64) & 0xFFFF) * 31
            + (np.round(y0 * 1000).astype(np.int64) & 0xFFFF))


def align_by_counter(bag, ds, seed=None):
    """dSPACE가 counter를 로깅한 경우 — 틱 단위 정확 정렬.

    오프셋 후보를 두 갈래로 모아 **검산 점수가 높은 것**을 고른다:
      ⓐ 지문 투표 — dSPACE가 state·ref0 까지 찍었을 때. 시드 없이 단독으로 잡힌다.
      ⓑ 시드 — v_ref 상호상관으로 얻은 대략의 시각(±수 틱)을 counter 오프셋으로 환산.
        dSPACE가 counter·v_ref만 찍었을 때(실제 rec1_024가 이 경우)의 경로.
    어느 쪽이든 **PC가 보낸 v_ref와 dSPACE가 받은 v_ref가 틱마다 같은지**로 검산한다.
    """
    if "counter" not in ds or "v_cmd" not in ds:
        return None
    ok = np.isfinite(ds["counter"])
    if ok.sum() < 50:
        return None
    c = np.round(ds["counter"][ok]).astype(np.int64)
    tds, vds = ds["t"][ok], ds["v_cmd"][ok]
    c = c + np.cumsum(np.concatenate([[0], (np.diff(c) < -30000).astype(np.int64)])) * 65536
    # counter가 바뀌는 지점만 (dSPACE가 10ms보다 빠르게 찍으면 같은 값이 반복된다)
    keep = np.concatenate([[True], np.diff(c) != 0])
    dup = int(len(c) - keep.sum())
    c, tds, vds = c[keep], tds[keep], vds[keep]
    gaps = int(np.sum(np.diff(c) - 1))          # 건너뛴 틱 = PC 송신 누락/CAN 유실
    n_bag = len(bag["t"])

    cands = []
    if seed is not None:
        est = np.round((seed["a"] * tds + seed["b"] - bag["t"][0]) / TICK).astype(np.int64)
        offs, cnts = np.unique(c - est, return_counts=True)
        cands += [int(o) for o in offs[np.argsort(-cnts)[:5]]]
    if "state" in ds or "x0" in ds:             # 지문 투표 (있는 열로만)
        zb, zd = np.zeros(n_bag), np.zeros(len(c))
        gs = (np.round(ds["state"][ok][keep]).astype(int) if "state" in ds
              else np.zeros(len(c), int))
        fp_bag = _fingerprint(bag["state"] if "state" in ds else np.zeros(n_bag, int),
                              bag["v_ref"], bag["x0"] if "x0" in ds else zb,
                              bag["y0"] if "y0" in ds else zb)
        fp_ds = _fingerprint(gs, vds, ds["x0"][ok][keep] if "x0" in ds else zd,
                             ds["y0"][ok][keep] if "y0" in ds else zd)
        table = {}
        for i, f in enumerate(fp_bag):
            table.setdefault(f, []).append(i)
        votes = {}
        for k in range(0, len(c), max(1, len(c) // 400)):
            for i in table.get(fp_ds[k], [])[:8]:
                votes[int(c[k] - i)] = votes.get(int(c[k] - i), 0) + 1
        cands += [o for o, _ in sorted(votes.items(), key=lambda kv: -kv[1])[:3]]

    best = None
    for off in dict.fromkeys(cands):
        idx = c - off
        inb = (idx >= 0) & (idx < n_bag)
        if inb.sum() < 50:
            continue
        # 검산: PC가 보낸 v_ref == dSPACE가 받은 v_ref (와이어 양자화 1mm/s)
        rate = float(np.mean(np.abs(bag["v_ref"][idx[inb]] - vds[inb]) <= 0.0015))
        if best is None or (rate, inb.sum()) > (best[1], best[2]):
            best = (off, rate, int(inb.sum()))
    if best is None:
        return None
    off, rate, n = best
    idx = c - off
    inb = (idx >= 0) & (idx < n_bag)
    a, b = np.polyfit(tds[inb], bag["t"][idx[inb]], 1)
    resid = bag["t"][idx[inb]] - (a * tds[inb] + b)
    return dict(method="counter", a=a, b=b, n=n, offset=off, match_rate=rate,
                match_keys="v_ref", resid_ms=float(np.std(resid) * 1e3),
                drift_ppm=float((a - 1) * 1e6), dup=dup, gaps=gaps)


def _xcorr_lag(x, y):
    """x(bag) 대비 y(dSPACE)의 지연 [샘플] + 정규화 상관 피크·2위 대비 여유."""
    x = x - x.mean()
    y = y - y.mean()
    n = 1 << int(np.ceil(np.log2(len(x) + len(y))))
    r = np.fft.irfft(np.fft.rfft(x, n) * np.conj(np.fft.rfft(y, n)), n)
    r = np.concatenate([r[-(len(y) - 1):], r[:len(x)]])       # lag = -(len(y)-1) .. len(x)-1
    lags = np.arange(-(len(y) - 1), len(x))
    denom = np.linalg.norm(x) * np.linalg.norm(y) or 1.0
    r = r / denom
    k = int(np.argmax(r))
    mask = np.abs(lags - lags[k]) > 50
    second = float(np.max(r[mask])) if mask.any() else 0.0
    lag = float(lags[k])
    if 0 < k < len(r) - 1:      # 포물선 보간 — 10ms 격자보다 잘게 (서브샘플)
        den = r[k - 1] - 2 * r[k] + r[k + 1]
        if den != 0:
            lag += float(np.clip(0.5 * (r[k - 1] - r[k + 1]) / den, -1, 1))
    return lag, float(r[k]), second


def _window_drift(x, y, lag0, windows=8, tol=300):
    """창별 지연을 재고 직선 기울기 = 클럭 드리프트 [샘플/샘플]. (기울기, 쓴 창 수)

    거친 지연 lag0 을 이미 알고 있으므로 **절대 상관 세기로 거르지 않는다** — lag0
    ±tol(기본 3s) 안에 떨어진 창만 채택한다. 조향은 구간마다 활동량이 달라(직선 구간은
    거의 0) 세기 문턱으로 거르면 창이 3개도 안 남는다."""
    seg_n = len(y) // windows
    pts = []
    for w in range(windows):
        seg = y[w * seg_n:(w + 1) * seg_n]
        if len(seg) < 200 or np.std(seg) < 1e-9:
            continue
        lg, pk, sc = _xcorr_lag(x, seg)
        lg -= w * seg_n
        if abs(lg - lag0) > tol or pk < 0.12 or pk < 1.3 * sc:
            continue
        pts.append((w * seg_n + seg_n / 2, lg))
    if len(pts) < 3:
        return float("nan"), len(pts), float("nan")
    c = np.array([p[0] for p in pts], float)
    lv = np.array([p[1] for p in pts], float)
    s, i0 = np.polyfit(c, lv, 1)
    if abs(s) > 1e-3 or np.std(lv - (s * c + i0)) > 20:   # 1000ppm↑·잔차 0.2s↑ = 오매칭
        return float("nan"), len(pts), float("nan")
    return float(s), len(pts), float(i0)


def align_by_vref(bag, ds):
    """dSPACE가 수신한 v_ref 와 PC가 보낸 v_ref 의 상호상관 — counter 없을 때의 표준.

    **원신호가 아니라 변화율(diff)로 상관을 잡는다.** v_ref는 대부분 0.60 평탄구간이라
    원신호로 상관하면 피크가 뭉개져(실측 피크 0.602 vs 2위 0.600) 200초를 헛짚는다.
    diff는 rate limit 램프·정지·감속만 남겨 시각 지문이 된다."""
    if "v_cmd" not in ds:
        return None
    t0 = ds["t"][0]
    grid_ds = np.arange(0, ds["t"][-1] - t0, TICK)
    y = np.diff(np.interp(grid_ds, ds["t"] - t0, ds["v_cmd"]))
    x = np.diff(np.interp(np.arange(0, bag["t"][-1] - bag["t"][0], TICK),
                          bag["t"] - bag["t"][0], bag["v_ref"]))
    if len(x) < 100 or len(y) < 100 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return None
    lag, peak, second = _xcorr_lag(x, y)
    # 클럭 드리프트는 **조향 신호**로 잰다 — v_ref는 평탄구간이 길어 창 대부분이 무정보다.
    # 조향은 매 틱 변해서 창마다 지연을 잴 수 있다. str에 실린 응답 지연(1차 지연)은
    # 상수라 기울기에서 상쇄되므로 드리프트 추정에는 무해하다 (지연 자체는 v_ref로 잡았다).
    ds_str = next((k for k in ("str_cmd_deg", "str_act_deg") if k in ds), None)
    slope, nwin, lag0 = float("nan"), 0, float("nan")
    if ds_str:
        d0 = np.hypot(bag["x0"], bag["y0"])
        k0 = np.where(d0 > 0.05, 2 * bag["y0"] / np.maximum(d0, 0.05) ** 2, 0.0)
        xs = np.diff(np.interp(np.arange(0, bag["t"][-1] - bag["t"][0], TICK),
                               bag["t"] - bag["t"][0],
                               np.clip(np.degrees(np.arctan(L_WB * k0)),
                                       -MAX_STEER_DEG, MAX_STEER_DEG)))
        ys = np.diff(np.interp(grid_ds, ds["t"] - t0, ds[ds_str]))
        slope, nwin, lag0 = _window_drift(xs, ys, lag)
    a = 1.0 / (1.0 - slope) if np.isfinite(slope) else 1.0
    lag_used = lag0 if np.isfinite(lag0) else lag       # 드리프트 보정된 시작 시점 지연
    b = bag["t"][0] + a * (lag_used * TICK - t0)
    return dict(method="v_ref 상호상관", a=a, b=float(b), n=len(y), peak=peak, second=second,
                drift_ppm=(slope * 1e6 if np.isfinite(slope) else float("nan")),
                drift_windows=nwin, lag_s=lag_used * TICK)


# ───────────────────────────────────────────────────────────────── 병합 ──
def build_merged(bag, ds, align, str_sign=None):
    """bag 틱 격자(100Hz)에 dSPACE 신호를 얹은 딕셔너리."""
    out = dict(bag)
    out.pop("_counts", None)
    n = len(bag["t"])
    d = np.hypot(bag["x0"], bag["y0"])
    kappa = np.where(d > 0.05, 2 * bag["y0"] / np.maximum(d, 0.05) ** 2, 0.0)
    out["ref0_dist_m"] = d
    out["str_geom_deg"] = np.clip(np.degrees(np.arctan(L_WB * kappa)), -MAX_STEER_DEG, MAX_STEER_DEG)
    for k in ("v_cmd_ds", "v_act", "str_cmd_deg", "str_act_deg", "counter_ds"):
        out[k] = np.full(n, np.nan)
    if ds is None or align is None:
        return out
    t_ds_in_ros = align["a"] * ds["t"] + align["b"]
    order = np.argsort(t_ds_in_ros)
    tr = t_ds_in_ros[order]

    def put(dst, src):
        if src in ds:
            v = ds[src][order]
            ok = np.isfinite(v)
            if ok.sum() > 1:
                out[dst] = np.interp(bag["t"], tr[ok], v[ok], left=np.nan, right=np.nan)

    put("v_cmd_ds", "v_cmd")
    put("v_act", "v_act")
    put("str_cmd_deg", "str_cmd_deg")
    put("str_act_deg", "str_act_deg")
    put("counter_ds", "counter")

    # 조향 부호 규약: dSPACE 조향은 PC ref y와 **반대 부호**다 (CLAUDE.md §3, 손상민).
    # 규약을 외워 쓰지 않고 매번 상관으로 확인한다 — 모델이 바뀌면 조용히 틀리기 때문.
    ref = out["str_cmd_deg"] if np.isfinite(out["str_cmd_deg"]).any() else out["str_act_deg"]
    sel = np.isfinite(ref) & (np.abs(out["str_geom_deg"]) > 2.0)
    corr = (float(np.corrcoef(out["str_geom_deg"][sel], ref[sel])[0, 1])
            if sel.sum() > 100 else float("nan"))
    sign = str_sign if str_sign else (-1 if corr < 0 else 1)
    if sign < 0:
        out["str_cmd_deg"] = -out["str_cmd_deg"]
        out["str_act_deg"] = -out["str_act_deg"]
    out["_str_sign"], out["_str_corr"] = sign, corr
    return out


def segments(state, t):
    """스테이트 연속 구간 [(i0, i1, state)] — 표·색칠 공용."""
    out, s = [], 0
    for i in range(1, len(state) + 1):
        if i == len(state) or state[i] != state[s]:
            out.append((s, i - 1, int(state[s])))
            s = i
    return out


def write_merged_csv(m, path):
    cols = ["t_rel_s", "t_epoch_s", "state", "state_name", "v_cmd", "v_cmd_ds", "v_act",
            "str_geom_deg", "str_cmd_deg", "str_act_deg", "ref0_x", "ref0_y", "ref0_dist_m",
            "n_pts", "gps_cross_track_m", "gps_fix_quality", "lane_confidence",
            "avoid_obstacle_detected", "avoid_ttc", "avoid_maneuver_done", "counter_ds"]
    t0 = m["t"][0]
    src = dict(t_rel_s=m["t"] - t0, t_epoch_s=m["t"], state=m["state"], v_cmd=m["v_ref"],
               v_cmd_ds=m["v_cmd_ds"], v_act=m["v_act"], str_geom_deg=m["str_geom_deg"],
               str_cmd_deg=m["str_cmd_deg"], str_act_deg=m["str_act_deg"], ref0_x=m["x0"],
               ref0_y=m["y0"], ref0_dist_m=m["ref0_dist_m"], n_pts=m["n_pts"],
               gps_cross_track_m=m["gps_cross_track_m"], gps_fix_quality=m["gps_fix_quality"],
               lane_confidence=m["lane_confidence"],
               avoid_obstacle_detected=m["avoid_obstacle_detected"], avoid_ttc=m["avoid_ttc"],
               avoid_maneuver_done=m["avoid_maneuver_done"], counter_ds=m["counter_ds"])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(len(m["t"])):
            w.writerow([f"{src[c][i]:.4f}" if c != "state_name" else STATE_NAME.get(m["state"][i], "?")
                        for c in cols])


# ───────────────────────────────────────────────────────────────── 그래프 ──
def setup_mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.font_manager as fm
    import matplotlib.pyplot as plt
    for p in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
        if os.path.exists(p):
            fm.fontManager.addfont(p)
    fam = next((f for f in ("Noto Sans CJK JP", "NanumGothic")
                if any(f == x.name for x in fm.fontManager.ttflist)), "DejaVu Sans")
    plt.rcParams.update({
        "font.family": fam, "font.size": 10, "axes.unicode_minus": False,
        "axes.edgecolor": "#c9c8c3", "axes.linewidth": 0.8,
        "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 9, "ytick.labelsize": 9,
        "grid.color": "#e6e5e1", "grid.linewidth": 0.7,
        "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
        "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "legend.fontsize": 9,
    })
    return plt


def shade_states(ax, t, state, label_min_s=5.0):
    """스테이트 구간 배경색 — 이 그림의 주인공. 긴 구간엔 이름표도 (축 안쪽에) 얹는다."""
    for a, b, s in segments(state, t):
        ax.axvspan(t[a], t[b], color=SBG.get(s, "#eee"), lw=0, zorder=0)
        if t[b] - t[a] >= label_min_s:
            ax.annotate(STATE_NAME.get(s, "?"), ((t[a] + t[b]) / 2, 0.985),
                        xycoords=("data", "axes fraction"), ha="center", va="top",
                        fontsize=8.5, color=SCOL.get(s, INK3), zorder=6,
                        bbox=dict(boxstyle="round,pad=0.18", fc="#fcfcfb", ec="none", alpha=.72))
    for a, _, s in segments(state, t)[1:]:
        ax.axvline(t[a], color="#ffffff", lw=1.6, zorder=1)
        ax.axvline(t[a], color=SCOL.get(s, INK3), lw=0.8, alpha=.55, zorder=1)


def series_legend(ax, **kw):
    """계열 범례는 제목 줄 오른쪽(축 위)에 — 축 안은 스테이트 이름표 자리다."""
    leg = ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), **kw)
    ax.add_artist(leg)      # 뒤에 state_legend 가 또 legend를 부르면 이게 지워진다
    return leg


def state_legend(ax, states):
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor=SBG[s], edgecolor=SCOL[s], label=STATE_NAME[s])
                       for s in sorted(states)],
              loc="upper left", bbox_to_anchor=(0, -0.18), ncol=4,
              title="배경색 = 주행 스테이트", title_fontsize=9)


def pick_waypoints(lat, wp_dir="src/stack_gps/waypoints"):
    """이 run이 쓴 트랙 CSV 찾기 — 후보마다 궤적까지의 거리를 재서 stack_gps가 기록한
    `cross_track_m`과 맞는 것을 고른다 (추측이 아니라 검산으로 고른다)."""
    if lat is None or not os.path.isdir(wp_dir):
        return None
    R, lat0 = 6378137.0, math.radians(lat["lat"][0])
    vE = np.radians(lat["lon"] - lat["lon"][0]) * R * math.cos(lat0)
    vN = np.radians(lat["lat"] - lat["lat"][0]) * R
    k = slice(None, None, max(1, len(vE) // 300))
    best = None
    for name in sorted(os.listdir(wp_dir)):
        if not (name.startswith("waypoints_") and name.endswith(".csv")):
            continue
        rows = [r for r in csv.DictReader(open(os.path.join(wp_dir, name)))
                if r.get("quality") is None or int(r["quality"]) == 4]
        if len(rows) < 5:
            continue
        wlat = np.array([float(r["lat"]) for r in rows])
        wlon = np.array([float(r["lon"]) for r in rows])
        wE = np.radians(wlon - lat["lon"][0]) * R * math.cos(lat0)
        wN = np.radians(wlat - lat["lat"][0]) * R
        d = np.sqrt(np.min((vE[k][:, None] - wE) ** 2 + (vN[k][:, None] - wN) ** 2, axis=1))
        err = float(np.median(np.abs(d - lat["cross"][k])))
        if best is None or err < best[0]:
            best = (err, name, wE, wN)
    if best and best[0] < 0.15:
        return dict(name=best[1], E=best[2], N=best[3], err=best[0])
    return None


def make_state_panels(m, out_dir, title):
    """스테이트별 패널 — 그 스테이트 구간만 이어 붙여 "상위 목표 vs 하위 실제"를 본다."""
    plt = setup_mpl()
    t = m["t"] - m["t"][0]
    st = m["state"]
    states = [s for s in sorted(set(int(x) for x in st))
              if sum(b - a for a, b, ss in segments(st, t) if ss == s) > 50]
    if not states:
        return
    GAP = 1.0
    fig, axes = plt.subplots(len(states), 2, figsize=(13.5, 3.1 * len(states)),
                             squeeze=False, gridspec_kw=dict(hspace=0.45, wspace=0.16))
    keys = ["str_geom_deg", "str_cmd_deg", "str_act_deg", "v_ref", "v_act"]
    for r, s in enumerate(states):
        segs = [(a, b) for a, b, ss in segments(st, t) if ss == s and b - a >= 20]
        xs, parts, bounds, off = [], {k: [] for k in keys}, [], 0.0
        for a, b in segs:
            n = b - a + 1
            xs.append(off + np.arange(n) * TICK)
            for k in keys:
                parts[k].append(m[k][a:b + 1])
            off += n * TICK + GAP
            xs.append(np.array([off - GAP / 2]))
            for k in keys:
                parts[k].append(np.array([np.nan]))
            bounds.append(off - GAP / 2)
        x = np.concatenate(xs)
        p = {k: np.concatenate(v) for k, v in parts.items()}

        ax = axes[r][0]
        ax.plot(x, p["str_geom_deg"], color=C_GEOM, lw=1.0, label="상위 목표 δ (ref 기하)")
        if np.isfinite(p["str_cmd_deg"]).any():
            ax.plot(x, p["str_cmd_deg"], color=C_CMD, lw=1.5, label="MPC 명령 str")
        if np.isfinite(p["str_act_deg"]).any():
            ax.plot(x, p["str_act_deg"], color=C_ACT, lw=1.5, label="실제 str (dSPACE)")
        ax.set_ylabel("조향각 [deg]")
        ax2 = axes[r][1]
        ax2.plot(x, p["v_ref"], color=C_CMD, lw=1.6, label="상위 목표 v_ref")
        if np.isfinite(p["v_act"]).any():
            ax2.plot(x, p["v_act"], color=C_ACT, lw=1.6, label="실제 v (dSPACE)")
        ax2.axhline(0.5, color=C_WARN, lw=0.8, ls=":")
        ax2.set_ylabel("속도 [m/s]")

        # 정지 중(v_ref<0.1)은 이득 계산에서 뺀다 — 안 움직이는 차의 조향은 실현되지
        # 않는데(0.5 m/s 하한, CLAUDE.md §3 ①) 그게 스테이트 이득을 통째로 끌어내린다
        gsel = (np.isfinite(p["str_act_deg"]) & (np.abs(p["str_geom_deg"]) > 0.5)
                & (p["v_ref"] >= 0.1))
        gain = (100 * np.sum(p["str_act_deg"][gsel] * p["str_geom_deg"][gsel])
                / np.sum(p["str_geom_deg"][gsel] ** 2)) if gsel.sum() > 20 else float("nan")
        verr = np.nanmean(np.abs(p["v_act"] - p["v_ref"])) if np.isfinite(p["v_act"]).any() \
            else float("nan")
        for c, ax_ in enumerate((ax, ax2)):
            for bx in bounds[:-1]:
                ax_.axvline(bx, color=INK3, lw=0.7, ls=(0, (2, 3)), alpha=.6)
            ax_.grid(axis="y")
            ax_.set_axisbelow(True)
            ax_.margins(y=0.18)
            ax_.set_xlim(-GAP / 2, off - GAP / 2)
            if r == len(states) - 1:
                ax_.set_xlabel(f"{STATE_NAME.get(s,'?')} 구간만 이어 붙인 시간 [s]"
                               if c == 0 else "〃 (점선 = 구간 경계)")
            pass
        ax.set_title(f"{STATE_NAME.get(s,'?')} — 구간 {len(segs)}개 · 총 {off - GAP:.1f}s"
                     f" · 조향 실현 이득 {gain:.0f}% (주행 중)", loc="left", fontsize=11,
                     color=SCOL.get(s, INK))
        ax2.set_title(f"평균 속도오차 {verr:.3f} m/s", loc="left", fontsize=11, color=INK2)
        ax.spines["left"].set_color(SCOL.get(s, INK3))
        ax.spines["left"].set_linewidth(2.4)
    # 계열 범례는 열마다 하나씩, 그림 아래에 (행마다 반복하면 제목을 덮는다)
    for col, ax_ in enumerate(axes[0]):
        h, lb = ax_.get_legend_handles_labels()
        fig.legend(h, lb, loc="upper center", bbox_to_anchor=(0.29 + 0.44 * col, 0.045),
                   ncol=len(h), frameon=False, fontsize=9)
    fig.suptitle("스테이트별 상위 목표 vs 하위 실제", x=0.006, y=1.005,
                 ha="left", fontsize=12.5, color=INK)
    fig.savefig(os.path.join(out_dir, "5_per_state_cmd_vs_actual.png"), dpi=150,
                bbox_inches="tight")
    plt.close(fig)


def make_plots(m, lat, out_dir, title, has_ds):
    plt = setup_mpl()
    t = m["t"] - m["t"][0]
    st = m["state"]
    states = sorted(set(int(s) for s in st))
    have_v = np.isfinite(m["v_act"]).any()
    have_s = np.isfinite(m["str_act_deg"]).any()

    # ── 1. 종방향: 목표 속도 vs 실제 속도 ─────────────────────────────────
    fig, ax = plt.subplots(figsize=(13, 5.6))
    shade_states(ax, t, st)
    ax.plot(t, m["v_ref"], color=C_CMD, lw=1.8, label="목표 v_ref (PC)", zorder=3)
    if np.isfinite(m["v_cmd_ds"]).any():
        ax.plot(t, m["v_cmd_ds"], color=INK3, lw=1.0, ls="--", label="v_ref (수신)", zorder=2)
    if have_v:
        ax.plot(t, m["v_act"], color=C_ACT, lw=1.8, label="실제 v", zorder=4)
    ax.set_ylabel("속도 [m/s]")
    ax.set_xlabel("경과 시간 [s]")
    ax.axhline(0.5, color=C_WARN, lw=0.9, ls=":", zorder=2)
    ax.annotate("조향 응답 하한 0.5", (t[0], 0.5), xytext=(4, 3), textcoords="offset points",
                ha="left", fontsize=8, color=C_WARN)
    ax.grid(axis="y")
    ax.margins(y=0.16)
    ax.set_title("종방향: 목표 대비 실제 속도", loc="left", fontsize=12, pad=12)
    series_legend(ax, ncol=3)
    state_legend(ax, states)
    fig.savefig(os.path.join(out_dir, "1_speed_cmd_vs_actual.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ── 2. 횡방향: 조향 명령(기하→MPC) vs 실제 ─────────────────────────────
    fig, ax = plt.subplots(figsize=(13, 5.6))
    shade_states(ax, t, st)
    ax.plot(t, m["str_geom_deg"], color=C_GEOM, lw=1.1, label="ref 기하 δ (PC)", zorder=2)
    if np.isfinite(m["str_cmd_deg"]).any():
        ax.plot(t, m["str_cmd_deg"], color=C_CMD, lw=1.8, label="MPC 출력 str", zorder=3)
    if have_s:
        ax.plot(t, m["str_act_deg"], color=C_ACT, lw=1.8, label="실제 str", zorder=4)
    ax.axhline(0, color=INK3, lw=0.8)
    ax.set_ylabel("조향각 [deg]")
    ax.set_xlabel("경과 시간 [s]")
    ax.grid(axis="y")
    ax.margins(y=0.14)
    ax.set_title("횡방향: 조향 명령 대비 실현", loc="left", fontsize=12, pad=12)
    series_legend(ax, ncol=3)
    state_legend(ax, states)
    fig.savefig(os.path.join(out_dir, "2_steer_cmd_vs_actual.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ── 3. 스테이트별 요약 (막대) ──────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    names = [STATE_NAME.get(s, "?") for s in states]
    cols = [SCOL.get(s, INK3) for s in states]

    def per_state(arr, fn=np.nanmean):
        out = []
        for s in states:
            v = arr[st == s]
            v = v[np.isfinite(v)]
            out.append(fn(v) if len(v) else np.nan)
        return np.asarray(out)

    def gain(num, den):
        """실현 이득 = 원점을 지나는 최소자승 기울기. 틱별 비(比)의 평균은 작은 분모에서
        폭주한다 — CLAUDE.md §3 ③ 의 "명령 곡률의 10~55%만 실현"이 이 값이다."""
        out = []
        for s in states:
            sel = ((st == s) & np.isfinite(num) & np.isfinite(den)
                   & (np.abs(den) > 0.5) & (m["v_ref"] >= 0.1))
            out.append(100 * np.sum(num[sel] * den[sel]) / np.sum(den[sel] ** 2)
                       if sel.sum() > 20 else np.nan)
        return np.asarray(out)

    dur = per_state(np.ones_like(t), np.sum) * TICK
    panels = [("체류 시간 [s]", dur),
              ("평균 |조향| [deg]", per_state(np.abs(m["str_act_deg"] if have_s
                                                   else m["str_geom_deg"])))]
    if have_s:
        panels.append(("조향 실현 이득 [%] — 실제 str / PC 기하 δ (주행 중)",
                       gain(m["str_act_deg"], m["str_geom_deg"])))
    elif np.isfinite(m["str_cmd_deg"]).any():
        panels.append(("조향 실현 이득 [%] — MPC str / PC 기하 δ",
                       gain(m["str_cmd_deg"], m["str_geom_deg"])))
    else:
        panels.append(("평균 횡오차 [m]", per_state(m["gps_cross_track_m"])))
    for ax, (lab, val) in zip(axes, panels):
        ax.bar(names, val, color=cols, width=0.62)
        for i, v in enumerate(val):
            if np.isfinite(v):
                ax.annotate(f"{v:.2f}", (i, v), xytext=(0, 3), textcoords="offset points",
                            ha="center", fontsize=9, color=INK2)
        ax.set_title(lab, loc="left", fontsize=9.5, color=INK2)
        ax.grid(axis="y")
        ax.set_axisbelow(True)
    fig.suptitle("스테이트별 요약", x=0.005, ha="left", fontsize=12, color=INK)
    fig.savefig(os.path.join(out_dir, "3_per_state_summary.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)

    # ── 4. 궤적 (스테이트 색) ─────────────────────────────────────────────
    if lat is not None:
        R, lat0 = 6378137.0, math.radians(lat["lat"][0])
        E = np.radians(lat["lon"] - lat["lon"][0]) * R * math.cos(lat0)
        N = np.radians(lat["lat"] - lat["lat"][0]) * R
        sidx = np.searchsorted(m["t"], lat["t"], side="right") - 1
        sidx = np.clip(sidx, 0, len(st) - 1)
        s_at = st[sidx]
        fig, ax = plt.subplots(figsize=(9, 9))
        wp = pick_waypoints(lat)
        if wp:
            ax.plot(wp["E"], wp["N"], "-", color=C_TRACK, lw=5.2, alpha=.42,
                    solid_capstyle="round", zorder=1, label=f"트랙 ({wp['name']})")
            ax.add_artist(ax.legend(loc="upper left", fontsize=8.5))
        for a, b, s in segments(s_at, lat["t"]):
            ax.plot(E[a:min(b + 2, len(E))], N[a:min(b + 2, len(N))], "-", lw=2.4,
                    color=SCOL.get(s, INK3), solid_capstyle="round", zorder=3)
        ax.plot(E[0], N[0], "o", ms=10, mfc="white", mec=C_TRACK, mew=2.2)
        ax.plot(E[-1], N[-1], "s", ms=10, mfc="white", mec=C_TRACK, mew=2.2)
        ax.set_aspect("equal")
        ax.set_xlabel("동 [m]")
        ax.set_ylabel("북 [m]")
        ax.grid(True)
        ax.set_axisbelow(True)
        ax.set_title("궤적 (색 = 주행 스테이트)", loc="left", fontsize=12)
        state_legend(ax, sorted(set(int(x) for x in s_at)))
        fig.savefig(os.path.join(out_dir, "4_trajectory_by_state.png"), dpi=150,
                    bbox_inches="tight")
        plt.close(fig)


# ───────────────────────────────────────────────────────────────── 리포트 ──
def report(bag, m, align, ds, out_dir):
    L = []
    t = m["t"] - m["t"][0]
    L.append(f"틱 {len(t)}개 · {t[-1]:.1f}s · 토픽 수신: {bag['_counts']}")
    if align is None:
        L.append("dSPACE 로그 없음 — bag 단독 분석 (실제 str/v 열은 비어 있음)")
    else:
        L.append(f"정렬: {align['method']}  t_ros = {align['a']:.9f}·t_ds + {align['b']:.3f}")
        if align["method"] == "counter":
            L.append(f"  대응 틱 {align['n']}개 · 지문({align['match_keys']}) 일치율 "
                     f"{align['match_rate']*100:.1f}% · 잔차 {align['resid_ms']:.2f} ms"
                     f" · 드리프트 {align['drift_ppm']:.0f} ppm")
            L.append(f"  counter 오프셋 {align['offset']} (bag_index = counter − off)"
                     f" · 중복 샘플 {align['dup']}개 · 건너뛴 틱 {align['gaps']}개")
            if align["match_rate"] < 0.9:
                L.append("  ⚠ 지문 일치율이 낮다 — dSPACE가 받은 값과 PC가 보낸 값이 다르다는 뜻."
                         " CAN 유실·스케일 불일치(PROTOCOL.md) 를 먼저 의심할 것")
        else:
            L.append(f"  지연 {align['lag_s']:+.3f}s · 상관 피크 {align['peak']:.3f}"
                     f" (2위 {align['second']:.3f}) · 드리프트 {align['drift_ppm']:.0f} ppm")
            if align["peak"] < 0.7 or align["peak"] - align["second"] < 0.1:
                L.append("  ⚠ 상관이 약하다 — 정렬을 믿지 말 것. counter 로깅을 요청하거나"
                         " --lag 로 수동 지정")
        L.append(f"  dSPACE 열 매핑: {ds['_map']}")
    L.append("")
    L.append("스테이트 구간 (0.3s 이상)")
    L.append(f"{'구간':>12} {'시작[s]':>8} {'끝[s]':>8} {'길이[s]':>7} "
             f"{'v목표':>6} {'v실제':>6} {'|δ명령|':>7} {'|δ실제|':>7} {'실현율':>6} {'횡오차':>6}")
    for a, b, s in segments(m["state"], t):
        if (b - a) * TICK < 0.3:
            continue
        sl = slice(a, b + 1)
        f = lambda x: np.nanmean(x[sl]) if np.isfinite(x[sl]).any() else float("nan")  # noqa: E731
        cmd = np.nanmean(np.abs(m["str_cmd_deg"][sl])) if np.isfinite(m["str_cmd_deg"][sl]).any() \
            else np.nanmean(np.abs(m["str_geom_deg"][sl]))
        act = np.nanmean(np.abs(m["str_act_deg"][sl]))
        L.append(f"{STATE_NAME.get(s,'?'):>12} {t[a]:8.2f} {t[b]:8.2f} {(t[b]-t[a]):7.2f} "
                 f"{f(m['v_ref']):6.2f} {f(m['v_act']):6.2f} {cmd:7.2f} {act:7.2f} "
                 f"{(act/cmd*100 if cmd > 0.5 and np.isfinite(act) else float('nan')):6.1f} "
                 f"{f(m['gps_cross_track_m']):6.2f}")
    txt = "\n".join(L)
    open(os.path.join(out_dir, "report.txt"), "w").write(txt + "\n")
    return txt


# ─────────────────────────────────────────────────────── 파이프라인 자체점검 ──
def synth_dspace(bag, path, lag_s=12.345, drift_ppm=180.0, tau=0.25, dt=0.001):
    """bag의 명령에 1차 지연·클럭 오차를 입혀 가짜 dSPACE CSV 생성 (정렬 검증용)."""
    t0 = bag["t"][0]
    n = int((bag["t"][-1] - t0) / dt)
    tds = np.arange(n) * dt
    t_ros = t0 + lag_s + tds * (1 + drift_ppm * 1e-6)
    v_cmd = np.interp(t_ros, bag["t"], bag["v_ref"])
    state = np.interp(t_ros, bag["t"], bag["state"]).round()
    ctr = np.clip(np.searchsorted(bag["t"], t_ros) + 1000, 0, None) % 65536
    d = np.hypot(bag["x0"], bag["y0"])
    k = np.where(d > 0.05, 2 * bag["y0"] / np.maximum(d, 0.05) ** 2, 0.0)
    sc = np.interp(t_ros, bag["t"], np.clip(np.degrees(np.arctan(L_WB * k)),
                                            -MAX_STEER_DEG, MAX_STEER_DEG)) * 0.55
    a = dt / (tau + dt)
    v_act = np.zeros(n)
    s_act = np.zeros(n)
    for i in range(1, n):
        v_act[i] = v_act[i - 1] + a * (v_cmd[i] - v_act[i - 1])
        s_act[i] = s_act[i - 1] + a * (sc[i] - s_act[i - 1])
    with open(path, "w", newline="") as f:
        f.write("# dSPACE ControlDesk export (synthetic)\n# run: selftest\n")
        w = csv.writer(f)
        w.writerow(["Time", "CAN_RX/counter", "CAN_RX/state", "CAN_RX/v_ref",
                    "Vehicle/v_meas", "MPC/str_ref", "Steer/str_meas"])
        for i in range(n):
            w.writerow([f"{tds[i]:.4f}", int(ctr[i]), int(state[i]), f"{v_cmd[i]:.4f}",
                        f"{v_act[i]:.4f}", f"{sc[i]:.4f}", f"{s_act[i]:.4f}"])
    return dict(lag_s=lag_s, drift_ppm=drift_ppm)


def main():
    ap = argparse.ArgumentParser(description="dSPACE 로그 × rosbag 병합 분석")
    ap.add_argument("run", nargs="?", help="drive_logs/run_<시각> 디렉터리")
    ap.add_argument("--dspace", help="dSPACE CSV")
    ap.add_argument("--map", default="", help='열 매핑 "t=Time,v_act=v_meas,..." (자동추정 보완)')
    ap.add_argument("--str-unit", choices=["deg", "rad"], help="dSPACE 조향 단위 (기본: 자동)")
    ap.add_argument("--v-unit", choices=["kmh", "mps"], help="dSPACE 속도 단위 (기본: 자동)")
    ap.add_argument("--str-sign", type=int, choices=[1, -1],
                    help="dSPACE 조향 부호 (기본: 상관으로 자동)")
    ap.add_argument("--lag", type=float, help="수동 정렬: t_ros = t_ds + lag [s]")
    ap.add_argument("--out", help="출력 디렉터리 (기본 <run>/analysis_dspace)")
    ap.add_argument("--list-columns", metavar="CSV", help="dSPACE CSV 열 이름만 출력하고 종료")
    ap.add_argument("--synth", metavar="CSV", help="bag에서 가짜 dSPACE CSV 생성 후 정렬 자체점검")
    args = ap.parse_args()

    if args.list_columns:
        header, data = sniff_dspace(args.list_columns)
        print(f"{len(header)}열 × {len(data)}행")
        for i, h in enumerate(header):
            print(f"  [{i:2d}] {h}")
        return

    if not args.run:
        ap.error("run 디렉터리를 지정하세요")
    run = args.run.rstrip("/")
    out_dir = args.out or os.path.join(run, "analysis_dspace")
    os.makedirs(out_dir, exist_ok=True)

    bag = read_bag(run)
    lat = read_lateral(run)

    truth = None
    dspace_csv = args.dspace
    if args.synth:
        truth = synth_dspace(bag, args.synth)
        dspace_csv = args.synth
        print(f"[selftest] 가짜 dSPACE CSV 생성: {args.synth} (진짜 지연 {truth['lag_s']}s)")

    ds = align = None
    if dspace_csv:
        umap = dict(kv.split("=", 1) for kv in args.map.split(",") if "=" in kv)
        ds = read_dspace(dspace_csv, umap, args.str_unit)
        ds["_v_unit"] = harmonize_speed(bag, ds, args.v_unit)
        if args.lag is not None:
            align = dict(method="수동(--lag)", a=1.0, b=args.lag, n=len(ds["t"]))
        else:
            seed = align_by_vref(bag, ds)
            align = align_by_counter(bag, ds, seed) or seed
        if align is None:
            sys.exit("[정렬] 실패 — counter·v_ref 어느 쪽으로도 못 맞췄다. --lag 로 수동 지정")

    m = build_merged(bag, ds, align, args.str_sign)
    write_merged_csv(m, os.path.join(out_dir, "merged.csv"))
    txt = report(bag, m, align, ds, out_dir)
    print(txt)
    if truth:
        est = align["b"] - bag["t"][0]        # t_ros = a·t_ds + b, 가짜 로그는 t_ds=0 이 시작
        print(f"\n[selftest] 진짜 지연 {truth['lag_s']:.3f}s → 추정 {est:.3f}s "
              f"(오차 {abs(est - truth['lag_s'])*1e3:.1f} ms), "
              f"드리프트 진짜 {truth['drift_ppm']:.0f} → "
              f"추정 {align.get('drift_ppm', float('nan')):.0f} ppm")
    make_plots(m, lat, out_dir, os.path.basename(run), ds is not None)
    make_state_panels(m, out_dir, os.path.basename(run))
    print(f"\n출력: {out_dir}/  (merged.csv, report.txt, *.png)")


if __name__ == "__main__":
    main()
