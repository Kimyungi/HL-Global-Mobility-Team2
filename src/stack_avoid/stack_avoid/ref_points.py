#!/usr/bin/env python3
"""ref 점 생성 — `avoid_to_ref` 와 `step_injector` 의 **공용 단일 소스**.  이기돈

★ 왜 공용 모듈인가 (팀장 리뷰 2026-08-10 ⑦).
  `step_injector` docstring 은 "avoid_to_ref 와 똑같은 계산" 이라고 주장했지만, 복제한
  것은 이미 삭제된 구방식(목표점까지의 등곡률 호 위 0.4m lookahead 점)이었다. 현재
  `avoid_to_ref` 기본은 `ray_pull` — **초록점 방향을 보존하고 거리만 당기는 직선 반직선**
  이고 yaw 는 0 이다. 두 계산은 같은 목표에 대해 다른 점을 낸다.
  그 상태로 잰 ①(조향 응답)·②(측방 이동 곡선)은 실제 회피 기동에 이전되지 않는다.
  → 계산을 여기 한 곳에 두고 두 노드가 import 한다 (estop_gate.py 와 같은 패턴).

이 모듈은 **ROS 의존이 없다** — 순수 기하. RefPoint 변환은 호출부에서 한다.
"""
import math


def ray_points(px, py, *, lookahead_m, n_points=1, spacing_m=0.32,
               min_turn_radius_m=1.15, ref_x_min_m=0.8,
               lat_sign=1.0, yaw_gain=0.0, curv_gain=1.0, y_scale=1.0):
    """목표점 (px, py) **방향을 보존**한 반직선 위에 ref 점을 생성한다.

    `avoid_to_ref._ray_points` + `_apply_gains` 를 합친 것으로, 기본 인자에서
    기존 동작과 완전히 동일하다(회귀 검증됨).

    왜 방향 보존인가 — dSPACE 는 REF_POINT_00 하나만 디코딩하고 그 점의 **방향각**이
    조향을 지배한다(F-11 상관 −0.95). 같은 각도라도 거리가 조향 크기를 정하므로
    (미리보기 = N_p×Ts×v_ref), 방향은 그대로 두고 거리만 팀 규약(lookahead_m)으로
    당긴다. MPC 튜닝이 아니라 다른 스택과 같은 ref 규약으로 맞추는 것이다.

    반환: [(x, y, yaw, curvature), ...] — 부호·게인이 모두 적용된 최종 값.
    """
    d = math.hypot(px, py)
    th = math.atan2(py, px)                      # ★ 보존 대상
    # 점 0 은 목표점보다 멀리 두지 않는다(장애물 너머를 가리키면 안 된다).
    # 다만 ref_x_min 아래로도 두지 않는다(너무 가까우면 quintic 이 무너진다).
    L0 = max(min(lookahead_m, d), ref_x_min_m)
    k_max = 1.0 / min_turn_radius_m
    c, s = math.cos(th), math.sin(th)
    out = []
    for i in range(max(1, int(n_points))):
        L = L0 + i * spacing_m
        x, y = L * c, L * s
        out.append(_apply_gains(x, y, th, L, k_max, lat_sign,
                                yaw_gain, curv_gain, y_scale))
    return out


def _apply_gains(x, y, th, L, k_max, lat_sign, yaw_gain, curv_gain, y_scale):
    """횡방향 부호·진단 게인 적용 후 (x, y, yaw, curvature).

    부호는 횡방향 성분(y·yaw·curvature)에만 — x(전방거리)는 그대로.
    y_scale 을 쓰면 축소된 점 기준으로 κ 를 재계산해 자기정합을 유지한다.
    """
    y2 = y * y_scale
    if abs(y_scale - 1.0) > 1e-9:
        d2 = x * x + y2 * y2
        kappa = max(-k_max, min(k_max, 2.0 * y2 / d2)) if d2 > 1e-9 else 0.0
    else:
        # 원점→그 점 등곡률 호의 곡률. H 절에서 조향에 무영향으로 확정됐으나
        # 점마다 자기정합은 유지한다.
        kappa = max(-k_max, min(k_max, 2.0 * y / (L * L)))
    return (x, lat_sign * y2, lat_sign * th * yaw_gain, lat_sign * kappa * curv_gain)


def lateral_step_point(offset_m, *, target_x_m, **kw):
    """측방 스텝 목표(offset_m)를 **회피와 같은 규약**의 ref 점으로.

    `step_injector` 전용 진입점. "target_x 앞, 옆으로 offset" 이라는 목표를 만들어
    `ray_points` 에 그대로 넘긴다 — 즉 스텝 시험이 내는 ref 는 실제 회피가 내는 ref 와
    같은 생성기를 거친다. 여기서 잰 조향 응답·측방 이동이 회피에 그대로 이전된다.
    """
    return ray_points(target_x_m, offset_m, **kw)
