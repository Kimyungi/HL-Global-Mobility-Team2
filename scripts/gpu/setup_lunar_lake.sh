#!/usr/bin/env bash
# Experimental Xe enablement for this machine; does not reboot or reload drivers.
set -euo pipefail
MODE="${1:-}"
CONF=/etc/default/grub.d/99-fma-lunar-lake.cfg
case "$MODE" in
  --apply|--rollback) ;;
  *) echo 'Usage: sudo bash setup_lunar_lake.sh --apply|--rollback' >&2; exit 2 ;;
esac
[[ $EUID == 0 ]] || { echo 'Administrator authentication required.' >&2; exit 1; }
if [[ "$MODE" == --rollback ]]; then
  if [[ -f "$CONF" ]]; then
    grep -qx '# Managed by FMA setup_lunar_lake.sh' "$CONF" || exit 1
    rm -- "$CONF"
  fi
  update-grub
  echo 'Force-probe removed. Reboot when ready. Added render/video group membership is retained.'
  exit 0
fi
echo 'Disabled: force_probe on this machine caused a boot hang on 2026-09-08.' >&2
echo 'Use --rollback to remove the old configuration. Reassess supported kernel/OS options for GPU enablement.' >&2
exit 1
