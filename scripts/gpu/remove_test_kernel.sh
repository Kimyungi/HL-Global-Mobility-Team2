#!/usr/bin/env bash
set -euo pipefail
if [[ ${EUID} -ne 0 ]]; then
  echo "Administrator authentication required." >&2
  exit 1
fi
rm -f /etc/default/grub.d/98-fma-kernel-test.cfg
apt-get remove --purge -y \
  linux-image-6.14.0-37-generic \
  linux-modules-6.14.0-37-generic \
  linux-modules-extra-6.14.0-37-generic
update-grub
echo "6.14 test kernel removed; Lunar Lake firmware files were retained."
