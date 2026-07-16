#!/usr/bin/env bash
set -uo pipefail

REPO_DIR=${REPO_DIR:-/home/waverover/ros2_ws/src}
WORKSPACE_DIR=${WORKSPACE_DIR:-/home/waverover/ros2_ws}
IDENTITY_FILE=${IDENTITY_FILE:-${REPO_DIR}/waverover/config/robot_identity.yaml}
REMOTE=${REMOTE:-origin}
BRANCH=${BRANCH:-main}
EXPECTED_REPOSITORY=${EXPECTED_REPOSITORY:-Yacine-Derder/Waverover-SIJ}
EXPECTED_REMOTE_URL=${EXPECTED_REMOTE_URL:-}
NETWORK_TIMEOUT_SEC=${NETWORK_TIMEOUT_SEC:-45}
LOCK_FILE=${LOCK_FILE:-${WORKSPACE_DIR}/.waverover-update.lock}
ROS_SETUP_FILE=${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}
BUILD_COMMAND=${BUILD_COMMAND:-colcon build --symlink-install}

log() { printf 'waverover-update: %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

if [[ $(id -u) -eq 0 ]]; then
    fail 'refusing to run as root'
fi
[[ -d "$WORKSPACE_DIR" ]] || fail "workspace does not exist: $WORKSPACE_DIR"
exec 9>"$LOCK_FILE" || fail "cannot open lock file: $LOCK_FILE"
flock -n 9 || fail 'another updater instance holds the lock'

git -C "$REPO_DIR" rev-parse --git-dir >/dev/null 2>&1 || \
    fail "not a Git repository: $REPO_DIR"
repo_top=$(git -C "$REPO_DIR" rev-parse --show-toplevel) || \
    fail 'cannot determine repository top level'
[[ $(realpath -m "$repo_top") == $(realpath -m "$REPO_DIR") ]] || \
    fail "repository top level is $repo_top, expected $REPO_DIR"

remote_url=$(git -C "$REPO_DIR" remote get-url "$REMOTE" 2>/dev/null) || \
    fail "missing configured remote: $REMOTE"
if [[ -n "$EXPECTED_REMOTE_URL" && "$remote_url" == "$EXPECTED_REMOTE_URL" ]]; then
    :
else
case "$remote_url" in
    https://github.com/${EXPECTED_REPOSITORY}|https://github.com/${EXPECTED_REPOSITORY}.git|\
    git@github.com:${EXPECTED_REPOSITORY}|git@github.com:${EXPECTED_REPOSITORY}.git|\
    ssh://git@github.com/${EXPECTED_REPOSITORY}|ssh://git@github.com/${EXPECTED_REPOSITORY}.git)
        ;;
    *) fail "unexpected $REMOTE URL: $remote_url" ;;
esac
fi

[[ -f "$IDENTITY_FILE" ]] || fail "identity file is missing: $IDENTITY_FILE"
identity_relative=$(realpath -m --relative-to="$REPO_DIR" "$IDENTITY_FILE")
[[ "$identity_relative" != ../* && "$identity_relative" != .. ]] || \
    fail "identity must be inside the repository: $IDENTITY_FILE"
if git -C "$REPO_DIR" ls-files --error-unmatch -- "$identity_relative" \
    >/dev/null 2>&1; then
    fail "identity is tracked by Git: $IDENTITY_FILE"
fi
git -C "$REPO_DIR" check-ignore -q -- "$identity_relative" || \
    fail "identity is not ignored by Git: $IDENTITY_FILE"
/usr/bin/python3 - "$IDENTITY_FILE" <<'PY' || fail "invalid identity: $IDENTITY_FILE"
import re
import sys
import yaml

path = sys.argv[1]
try:
    with open(path, encoding='utf-8') as stream:
        identity = yaml.safe_load(stream)
except (OSError, yaml.YAMLError) as error:
    print('waverover-update: identity error: %s' % error, file=sys.stderr)
    raise SystemExit(1)
if not isinstance(identity, dict) or set(identity) != {'robot_name'}:
    print('waverover-update: identity must contain only robot_name', file=sys.stderr)
    raise SystemExit(1)
name = str(identity['robot_name']).strip() if identity['robot_name'] is not None else ''
if not re.fullmatch(r'[A-Za-z0-9_]+', name):
    print('waverover-update: invalid robot_name', file=sys.stderr)
    raise SystemExit(1)
PY

[[ -z $(git -C "$REPO_DIR" status --porcelain) ]] || \
    fail 'working tree is dirty; refusing to update'
current_branch=$(git -C "$REPO_DIR" symbolic-ref --quiet --short HEAD) || \
    fail 'detached HEAD is not safe for automatic updates'
[[ "$current_branch" == "$BRANCH" ]] || \
    fail "current branch is $current_branch, expected $BRANCH"
git -C "$REPO_DIR" show-ref --verify --quiet "refs/heads/$BRANCH" || \
    fail "local branch does not exist: $BRANCH"

old_commit=$(git -C "$REPO_DIR" rev-parse "refs/heads/$BRANCH") || \
    fail 'cannot resolve current commit'
log "current commit: $old_commit"
if ! timeout "${NETWORK_TIMEOUT_SEC}s" git -C "$REPO_DIR" fetch \
    --no-tags "$REMOTE" "refs/heads/$BRANCH"; then
    log 'remote fetch unavailable; keeping the current software'
    exit 0
fi
target_commit=$(git -C "$REPO_DIR" rev-parse FETCH_HEAD) || \
    fail 'cannot resolve fetched target'
log "target commit: $target_commit"

build_workspace() {
    (
        cd "$WORKSPACE_DIR" || exit 1
        if [[ -n "$ROS_SETUP_FILE" ]]; then
            [[ -r "$ROS_SETUP_FILE" ]] || {
                log "ROS setup is unreadable: $ROS_SETUP_FILE"
                exit 1
            }
            # shellcheck disable=SC1090
            set +u
            source "$ROS_SETUP_FILE"
            setup_status=$?
            set -u
            if [[ $setup_status -ne 0 ]]; then
                log "failed to source ROS setup: $ROS_SETUP_FILE"
                exit 1
            fi
        fi
        bash -c "$BUILD_COMMAND"
    )
}

if [[ "$old_commit" == "$target_commit" ]]; then
    if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
        log 'already current and install/setup.bash exists; build skipped'
        exit 0
    fi
    log 'already current but install/setup.bash is missing; building'
    build_workspace || fail "build failed at current commit $old_commit"
    log "deployed commit: $old_commit"
    exit 0
fi

if git -C "$REPO_DIR" merge-base --is-ancestor "$target_commit" "$old_commit"; then
    fail 'local branch is ahead of the remote target; refusing to rewind'
fi
git -C "$REPO_DIR" merge-base --is-ancestor "$old_commit" "$target_commit" || \
    fail 'local and remote branches have diverged; refusing to update'

log "applying verified fast-forward $old_commit -> $target_commit"
git -C "$REPO_DIR" update-ref "refs/heads/$BRANCH" \
    "$target_commit" "$old_commit" || fail 'could not advance the branch ref'
if ! git -C "$REPO_DIR" read-tree --reset -u "$target_commit"; then
    git -C "$REPO_DIR" update-ref "refs/heads/$BRANCH" \
        "$old_commit" "$target_commit" || true
    git -C "$REPO_DIR" read-tree --reset -u "$old_commit" || true
    fail 'could not populate the fast-forward target tree'
fi

if build_workspace; then
    log "deployed commit: $target_commit"
    exit 0
fi

log "new build failed; rolling back to $old_commit" >&2
git -C "$REPO_DIR" update-ref "refs/heads/$BRANCH" \
    "$old_commit" "$target_commit" || \
    fail 'new build failed and the previous branch ref could not be restored'
git -C "$REPO_DIR" read-tree --reset -u "$old_commit" || \
    fail 'new build failed and the previous source tree could not be restored'
if build_workspace; then
    log "rollback build succeeded; deployed commit: $old_commit"
    exit 0
fi
fail "new build and rollback build both failed; source is at $old_commit"
