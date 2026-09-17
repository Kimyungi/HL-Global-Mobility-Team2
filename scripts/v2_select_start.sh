# Sourced by scripts/v2 before prepare/drive starts any launch actions.
# V2_LAUNCH_ARGS is the validated argument array passed to ros2 launch.
v2_select_start() {
  V2_LAUNCH_ARGS=("$@")
  local v2_arg v2_start='' v2_end='' v2_start_seen=0 v2_end_seen=0
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
      end_waypoint:=*)
        if (( v2_end_seen )); then
          echo 'end_waypoint는 한 번만 지정하세요.' >&2; return 2
        fi
        v2_end_seen=1; v2_end=${v2_arg#end_waypoint:=} ;;
    esac
  done
  if (( ! v2_start_seen )); then
    if [[ ! -t 0 ]]; then
      echo '시작 CSV를 선택해야 합니다. start_waypoint:=03으로 주차 시험 경로를 지정하세요.' >&2
      return 2
    fi
    echo '시작 CSV 선택: PR #116 경로 03 (parking_waypoint.csv)' >&2
    while :; do
      if ! read -r -p '시작 경로 번호 (기본값 없음): ' v2_start; then
        echo '시작 경로 입력이 종료되어 실행을 취소합니다.' >&2; return 2
      fi
      case "$v2_start" in
        3) v2_start="0$v2_start"; break ;;
        03) break ;;
        *) echo '주차 시험 경로 03을 입력하세요.' >&2 ;;
      esac
    done
    V2_LAUNCH_ARGS+=("start_waypoint:=$v2_start")
  fi
  case "$v2_start" in
    03) ;;
    *) echo 'start_waypoint는 주차 시험 경로 03이어야 합니다.' >&2; return 2 ;;
  esac
  if (( ! v2_end_seen )); then
    # PR #116 ends on its only GPS route.
    v2_end=03
    V2_LAUNCH_ARGS+=("end_waypoint:=$v2_end")
  fi
  case "$v2_end" in
    03) ;;
    *) echo 'end_waypoint는 03이어야 합니다.' >&2; return 2 ;;
  esac
  echo "[v2] 시작 경로: $v2_start / 종료 경로: $v2_end" >&2
}
