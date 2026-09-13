#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Administrator authentication required." >&2
  exit 1
fi

key=/tmp/intel-graphics.key
expected=E0258B57D9C442D5DB1855C271740E4DE392BFE3
actual=$(gpg --show-keys --with-colons "$key" | awk -F: '$1=="fpr" {print $10; exit}')
[[ "$actual" == "$expected" ]] || { echo "Unexpected Intel key fingerprint: $actual" >&2; exit 1; }
gpg --batch --yes --dearmor --output /usr/share/keyrings/intel-graphics.gpg "$key"
install -m 644 /dev/null /etc/apt/sources.list.d/intel-gpu-jammy.list
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy unified' > /etc/apt/sources.list.d/intel-gpu-jammy.list
apt-get update
echo "Intel Jammy GPU runtime repository added."
