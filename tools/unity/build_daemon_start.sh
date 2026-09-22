#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=tools/unity/build_daemon_lib.sh
source "${SCRIPT_DIR}/build_daemon_lib.sh"

SKIP_SYNC=0
WAIT_TIMEOUT_SEC=240

usage() {
    cat <<'EOF'
Usage:
  tools/unity/build_daemon_start.sh [options]

Start a persistent Unity Editor on the build worktree. Build requests can then be
sent with tools/unity/build_daemon_request.sh without paying Unity cold-start cost.

Options:
  --project-root PATH
  --worktree-path PATH
  --unity PATH
  --log-file PATH
  --skip-sync
  --wait-timeout-sec SEC
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --project-root) PROJECT_ROOT="$2"; shift 2 ;;
        --worktree-path) WORKTREE_PATH="$2"; shift 2 ;;
        --unity) UNITY_EDITOR="$2"; shift 2 ;;
        --log-file) DAEMON_LOG_FILE="$2"; shift 2 ;;
        --skip-sync) SKIP_SYNC=1; shift ;;
        --wait-timeout-sec) WAIT_TIMEOUT_SEC="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

require_cmd git
require_cmd rsync
require_cmd python3
normalize_paths
create_or_verify_worktree

if [[ "$SKIP_SYNC" -eq 0 ]]; then
    sync_forward
else
    printf '[sync] skipped by --skip-sync\n'
fi

init_daemon_env

if daemon_is_running; then
    printf '[daemon] already running for %s (pid=%s)\n' "$WORKTREE_PATH" "$(unity_process_for_worktree)"
    exit 0
fi

rm -f "$DAEMON_READY_FILE" "$DAEMON_REQUEST_FILE" "$DAEMON_RESULT_FILE"
printf '%s\n' "$DAEMON_LOG_FILE" > "$DAEMON_LOG_PATH_FILE"
[[ -x "$UNITY_EDITOR" ]] || die "Unity Editor executable not found or not executable: $UNITY_EDITOR"

printf '[daemon] starting Unity build daemon\n'
printf '[daemon] projectPath=%s\n' "$WORKTREE_PATH"
printf '[daemon] logFile=%s\n' "$DAEMON_LOG_FILE"

nohup "$UNITY_EDITOR" \
    -batchmode \
    -projectPath "$WORKTREE_PATH" \
    -executeMethod FinsSimBuildDaemon.Start \
    -logFile "$DAEMON_LOG_FILE" \
    >/tmp/finsim-unity-build-daemon.nohup 2>&1 &

echo "$!" > "$DAEMON_PID_FILE"

deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
while [[ "$SECONDS" -lt "$deadline" ]]; do
    if [[ -f "$DAEMON_READY_FILE" ]]; then
        printf '[daemon] ready (pid=%s)\n' "$(unity_process_for_worktree)"
        exit 0
    fi
    if ! daemon_is_running; then
        tail -n 120 "$DAEMON_LOG_FILE" 2>/dev/null || true
        die "Unity build daemon exited before becoming ready."
    fi
    sleep 1
done

tail -n 120 "$DAEMON_LOG_FILE" 2>/dev/null || true
die "Timed out waiting for Unity build daemon readiness after ${WAIT_TIMEOUT_SEC}s."
