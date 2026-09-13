#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Administrator authentication required." >&2
  exit 1
fi

config=/etc/gdm3/custom.conf
backup=/etc/gdm3/custom.conf.fma-xe-backup-20260909
[[ -f "$config" ]] || { echo "Missing $config" >&2; exit 1; }
if [[ ! -e "$backup" ]]; then
  cp -a "$config" "$backup"
fi

python3 - "$config" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
if "WaylandEnable=false" in text and "#WaylandEnable=false" not in text:
    raise SystemExit(0)
old = "#WaylandEnable=false"
if old not in text:
    raise SystemExit("Expected GDM Wayland setting was not found; refusing broad edit")
path.write_text(text.replace(old, "WaylandEnable=false", 1))
PY

echo "GDM will use Xorg on the next boot."
