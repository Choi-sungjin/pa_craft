#!/usr/bin/env bash
# Install (or remove) the clock restore system service. Files it replaces are kept next to them as *.bak-<time>.
#
#   sudo scripts/install_clock_restore.sh            # install and enable; works from the next power-on
#   sudo scripts/install_clock_restore.sh --remove   # disable and delete
set -e
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BIN=/usr/local/sbin/craft-clock-restore
UNIT=/etc/systemd/system/craft-clock-restore.service
[ "$(id -u)" = "0" ] || { echo "run with sudo" >&2; exit 1; }

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now craft-clock-restore.service 2>/dev/null || true
  rm -f "$BIN" "$UNIT"
  systemctl daemon-reload
  echo "removed"
  exit 0
fi

WHEN="$(date +%Y%m%d-%H%M%S)"
for target in "$BIN" "$UNIT"; do
  [ -e "$target" ] && cp -p "$target" "$target.bak-$WHEN" && echo "kept $target.bak-$WHEN"
done
install -m 0755 -o root -g root "$HERE/craft-clock-restore" "$BIN"
install -m 0644 -o root -g root "$HERE/craft-clock-restore.service" "$UNIT"
systemctl daemon-reload
systemctl enable craft-clock-restore.service
echo "installed: $BIN, $UNIT (enabled; runs at the next power-on)"
echo "check after a reboot: journalctl -b -u craft-clock-restore.service"
