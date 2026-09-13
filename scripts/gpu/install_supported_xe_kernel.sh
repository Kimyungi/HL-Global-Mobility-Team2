#!/usr/bin/env bash
# Install the pre-staged Ubuntu 6.14 kernel beside 6.8 and prepare a recoverable first boot.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Administrator authentication required." >&2
  exit 1
fi

stage=/tmp/fma-lnl-kernel-6.14.0-37
kernel_release=6.14.0-37-generic
grub_cfg=/etc/default/grub.d/98-fma-kernel-test.cfg

for file in \
  "$stage/linux-image-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb" \
  "$stage/linux-modules-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb" \
  "$stage/linux-modules-extra-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb"; do
  [[ -f "$file" ]] || { echo "Missing staged package: $file" >&2; exit 1; }
done

cd "$stage"
sha256sum -c - <<'EOF'
9c17bb3f50741bf8f21b0f8a2afd088f5d0ae5b0d1cc00b7a866a63c8492d29a  linux-image-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb
1bb1438d7d5f266da64f9ecc18f41bf37e5b94ad8a052b5d367ae09af03352fb  linux-modules-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb
e50be91ad2a10e7c2cb60254b7be098c6b402119240bb8a102737b3559c69157  linux-modules-extra-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb
5ebacb8a7e330eb97da02922eb30e49a1aabd4c84b37e4429f49598ffa709102  firmware/lnl_gsc_1.bin
d0c051eca7c6c94167931aed62fd5f0c0c9c8439a937217164240e9fae8d7ac1  firmware/lnl_guc_70.bin
320e765cbe42b0f5f95333794a6a00a4e67246d6ad8393b59b64efdc3506cfe9  firmware/lnl_huc.bin
a50837761a0039e9525bb0fffff44f2b9ecdf0304a1048101bd686fa7c548110  firmware/xe2lpd_dmc.bin
EOF

# These three firmware files are copied from the linux-firmware 20250509 tag.
# The installed Jammy firmware package predates Lunar Lake and cannot be replaced
# by Noble's split package because Jammy's initramfs-tools is too old.
install -d -m 755 /lib/firmware/xe
install -m 644 "$stage/firmware/lnl_gsc_1.bin" /lib/firmware/xe/lnl_gsc_1.bin
install -m 644 "$stage/firmware/lnl_guc_70.bin" /lib/firmware/xe/lnl_guc_70.bin
install -m 644 "$stage/firmware/lnl_huc.bin" /lib/firmware/xe/lnl_huc.bin
install -m 644 "$stage/firmware/xe2lpd_dmc.bin" /lib/firmware/i915/xe2lpd_dmc.bin

dpkg -i \
  "$stage/linux-modules-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb" \
  "$stage/linux-modules-extra-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb" \
  "$stage/linux-image-6.14.0-37-generic_6.14.0-37.37~24.04.1_amd64.deb"

# Always expose GRUB during the trial. The 6.8 kernel remains installed and can
# be selected from "Advanced options for Ubuntu" if 6.14 does not boot.
install -d -m 755 /etc/default/grub.d
cat > "$grub_cfg" <<'EOF'
# Managed by FMA install_supported_xe_kernel.sh
GRUB_TIMEOUT_STYLE=menu
GRUB_TIMEOUT=10
EOF
chmod 644 "$grub_cfg"
update-initramfs -u -k "$kernel_release"
update-grub

echo "Installed $kernel_release beside the existing 6.8 kernel."
echo "No force_probe and no nomodeset were added. Reboot manually when ready."
