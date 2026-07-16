#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=/home/waverover/ros2_ws/src
WORKSPACE_DIR=/home/waverover/ros2_ws
IDENTITY_FILE=$REPO_DIR/waverover/config/robot_identity.yaml
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

fail() { printf 'waverover installer: ERROR: %s\n' "$*" >&2; exit 1; }
repo_git() { git -c safe.directory="$REPO_DIR" -C "$REPO_DIR" "$@"; }
[[ $(id -u) -eq 0 ]] || fail 'run this installer explicitly with sudo'
[[ $(realpath -m "$SCRIPT_DIR/..") == $(realpath -m "$REPO_DIR") ]] || \
    fail "installer is not running from $REPO_DIR/deployment"
[[ -d "$WORKSPACE_DIR" && -d "$REPO_DIR/.git" ]] || \
    fail 'expected repository/workspace paths are missing'
[[ $(repo_git rev-parse --show-toplevel) == "$REPO_DIR" ]] || \
    fail 'repository top level does not match REPO_DIR'
[[ -z $(repo_git status --porcelain) ]] || \
    fail 'live repository is dirty; review and commit first'
[[ -f "$IDENTITY_FILE" ]] || fail 'robot_identity.yaml is missing'
if repo_git ls-files --error-unmatch \
    waverover/config/robot_identity.yaml >/dev/null 2>&1; then
    fail 'robot_identity.yaml is tracked'
fi
repo_git check-ignore -q waverover/config/robot_identity.yaml || \
    fail 'robot_identity.yaml is not ignored'
/usr/bin/python3 - "$IDENTITY_FILE" <<'PY' || fail 'robot identity is invalid'
import re
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    value = yaml.safe_load(stream)
if not isinstance(value, dict) or set(value) != {'robot_name'}:
    raise SystemExit(1)
name = str(value['robot_name']).strip() if value['robot_name'] is not None else ''
raise SystemExit(0 if re.fullmatch(r'[A-Za-z0-9_]+', name) else 1)
PY

install -d -o root -g root -m 0755 /usr/local/lib/waverover
install -o root -g root -m 0755 "$SCRIPT_DIR/update_and_build.sh" \
    /usr/local/lib/waverover/update_and_build.sh
install -o root -g root -m 0644 "$SCRIPT_DIR/waverover-update.service" \
    /etc/systemd/system/waverover-update.service
install -d -o root -g root -m 0755 /etc/systemd/system/waverover.service.d
install -o root -g root -m 0644 \
    "$SCRIPT_DIR/waverover.service.d/10-auto-update.conf" \
    /etc/systemd/system/waverover.service.d/10-auto-update.conf
if [[ ! -e /etc/default/waverover-update ]]; then
    install -o root -g root -m 0644 /dev/null /etc/default/waverover-update
fi
systemctl daemon-reload

printf '%s\n' \
    'Installed files only; no service or updater was started.' \
    'Rerun deployment/install.sh after deployment files change.' \
    'Manual updater test (stop the rover first):' \
    '  sudo systemctl stop waverover.service' \
    '  sudo systemctl start waverover-update.service' \
    '  journalctl -u waverover-update.service -n 200 --no-pager' \
    '  sudo systemctl start waverover.service' \
    'Logs:' \
    '  journalctl -u waverover-update.service -u waverover.service -b --no-pager' \
    'Disable/remove automatic updates:' \
    '  sudo rm /etc/systemd/system/waverover.service.d/10-auto-update.conf' \
    '  sudo rm /etc/systemd/system/waverover-update.service' \
    '  sudo rm /usr/local/lib/waverover/update_and_build.sh' \
    '  sudo systemctl daemon-reload'
