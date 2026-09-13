#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Administrator authentication required." >&2
  exit 1
fi

backup=/etc/gdm3/custom.conf.fma-xe-backup-20260909
if [[ -f "$backup" ]]; then
  install -m 644 "$backup" /etc/gdm3/custom.conf
fi
ppa-purge -y ppa:kisak/turtle
echo "Ubuntu Mesa packages and the previous GDM configuration were restored."
