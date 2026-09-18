# Sourced by scripts/v2 before prepare/drive starts any launch actions.
# V2_LAUNCH_ARGS is the validated argument array passed to ros2 launch.
v2_select_start() {
  V2_LAUNCH_ARGS=("$@")
  local v2_arg v2_start='' v2_end='' v2_start_seen=0 v2_end_seen=0 v2_speed='' v2_speed_seen=0
  for v2_arg in "$@"; do
    case "$v2_arg" in
      -h|--help|-s|--show-args|--show-arguments|-p|--print|--print-description)
        return 0 ;;
    esac
  done
  for v2_arg in "$@"; do
    case "$v2_arg" in
      start_waypoint:=*)
        if (( v2_start_seen )); then
          echo 'start_waypoint는 한 번만 지정하세요.' >&2; return 2
        fi
        v2_start_seen=1; v2_start=${v2_arg#start_waypoint:=} ;;
      v_base:=*)
        if (( v2_speed_seen )); then
          echo 'v_base는 한 번만 지정하세요.' >&2; return 2
        fi
        v2_speed_seen=1; v2_speed=${v2_arg#v_base:=} ;;
      end_waypoint:=*)
        if (( v2_end_seen )); then
          echo 'end_waypoint는 한 번만 지정하세요.' >&2; return 2
        fi
        v2_end_seen=1; v2_end=${v2_arg#end_waypoint:=} ;;
    esac
  done
  if (( ! v2_start_seen )); then
    if [[ ! -t 0 ]]; then
      echo '시작 CSV를 선택해야 합니다. start_waypoint:=01 등 01~07 경로를 지정하세요.' >&2
      return 2
    fi
    echo '시작 CSV 선택: halla_0919_path_01.csv ~ 07.csv' >&2
    while :; do
      if ! read -r -p '시작 경로 번호 (기본값 없음): ' v2_start; then
        echo '시작 경로 입력이 종료되어 실행을 취소합니다.' >&2; return 2
      fi
      case "$v2_start" in
        [1-7]) v2_start="0$v2_start"; break ;;
        0[1-7]) break ;;
        *) echo '01~07 경로를 입력하세요.' >&2 ;;
      esac
    done
    V2_LAUNCH_ARGS+=("start_waypoint:=$v2_start")
  fi
  case "$v2_start" in
    0[1-7]) ;;
    *) echo 'start_waypoint는 01~07이어야 합니다.' >&2; return 2 ;;
  esac
  if (( ! v2_end_seen )); then
    # Terminal branches are alternatives, not consecutive legs.
    v2_end=06
    if [[ "$v2_start" == 07 ]]; then v2_end=07; fi
    V2_LAUNCH_ARGS+=("end_waypoint:=$v2_end")
  fi
  case "$v2_end" in
    06|07) ;;
    *) echo 'end_waypoint는 06 또는 07이어야 합니다.' >&2; return 2 ;;
  esac
  if [[ "$v2_start" == 06 || "$v2_start" == 07 ]] && [[ "$v2_start" != "$v2_end" ]]; then
    echo "종료 경로에서 시작하면 start/end가 같아야 합니다." >&2; return 2
  fi
  if (( ! v2_speed_seen )); then
    if [[ ! -t 0 ]]; then
      echo '일반 주행 목표속도를 지정하세요. 예: v_base:=2.0 (m/s)' >&2; return 2
    fi
    while :; do
      if ! read -r -p '일반 주행 목표속도 v_base [m/s] (기본값 없음): ' v2_speed; then
        echo '속도 입력이 종료되어 실행을 취소합니다.' >&2; return 2
      fi
      if v2_valid_speed "$v2_speed"; then break; fi
      echo '0보다 큰 유한한 숫자를 입력하세요. 예: 2.0' >&2
    done
    V2_LAUNCH_ARGS+=("v_base:=$v2_speed")
  elif ! v2_valid_speed "$v2_speed"; then
    echo 'v_base는 0보다 큰 유한한 숫자여야 합니다.' >&2; return 2
  fi
  echo "[v2] 일반 주행 목표속도: $v2_speed m/s" >&2
  echo "[v2] 시작 경로: $v2_start / 종료 경로: $v2_end" >&2
}

# Numeric validation only; no ROS or hardware is started here.
v2_valid_speed() {
  python3 -c 'import math,sys
try:
    value=float(sys.argv[1])
    sys.exit(0 if math.isfinite(value) and value > 0 else 1)
except ValueError:
    sys.exit(1)' "$1"
}
