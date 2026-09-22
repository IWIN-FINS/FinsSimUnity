#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=tools/unity/build_daemon_lib.sh
source "${SCRIPT_DIR}/build_daemon_lib.sh"

FORCE=0

usage() {
    cat <<'EOF'
Usage:
  tools/unity/build_daemon_stop.sh [options]

Options:
  --project-root PATH
  --worktree-path PATH
  --force
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --project-root) PROJECT_ROOT="$2"; shift 2 ;;
        --worktree-path) WORKTREE_PATH="$2"; shift 2 ;;
        --force) FORCE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

require_cmd python3
normalize_paths
init_daemon_env

pid="$(unity_process_for_worktree || true)"
if [[ -z "$pid" ]]; then
    printf '[daemon] not running for %s\n' "$WORKTREE_PATH"
    exit 0
fi

if kill -0 "$pid" >/dev/null 2>&1; then
    printf '[daemon] terminating pid=%s%s\n' "$pid" "$([[ "$FORCE" -eq 1 ]] && printf ' (force)' || true)"
    kill -TERM "$pid" || true
    sleep 3
fi

if kill -0 "$pid" >/dev/null 2>&1; then
    printf '[daemon] killing pid=%s\n' "$pid"
    kill -KILL "$pid" || true
fi

rm -f "$DAEMON_PID_FILE" "$DAEMON_READY_FILE" "$DAEMON_REQUEST_FILE"
printf '[daemon] stopped\n'
