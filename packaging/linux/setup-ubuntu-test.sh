#!/usr/bin/env bash
# Provision a dedicated Ubuntu 24.04 test VM, not a general-purpose workstation.
set -euo pipefail
test "$(id -u)" = 0
test "$(uname -m)" = x86_64
test_user=${1:-rocket}
id "$test_user" >/dev/null
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  xfce4 xfce4-terminal lightdm xserver-xorg dbus-x11 \
  qemu-guest-agent spice-vdagent xvfb xauth mesa-utils \
  libgtk-3-0t64 libgl1 libglx-mesa0 libgl1-mesa-dri libxtst6 libxi6 \
  fonts-dejavu-core fakeroot binutils dpkg-dev desktop-file-utils python3 imagemagick
install -d /etc/lightdm/lightdm.conf.d
cat > /etc/lightdm/lightdm.conf.d/90-openrocket-test.conf <<EOF
[Seat:*]
autologin-user=$test_user
autologin-user-timeout=0
user-session=xfce
EOF
systemctl enable --now qemu-guest-agent
systemctl set-default graphical.target
systemctl enable --now lightdm
printf 'OPENROCKET_UBUNTU_SETUP=PASS\n'
