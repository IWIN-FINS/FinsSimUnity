#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=tools/unity/build_daemon_lib.sh
source "${SCRIPT_DIR}/build_daemon_lib.sh"

TASK_ID=""
SKIP_SYNC=0
SYNC_BACK=0
START_IF_NEEDED=1
WAIT_TIMEOUT_SEC=1800
RESULT_GRACE_SEC=30
VERBOSE=0
PROGRESS_INTERVAL_SEC=15

usage() {
    cat <<'EOF'
Usage:
  tools/unity/build_daemon_request.sh TASK_ID [options]
  tools/unity/build_daemon_request.sh --task TASK_ID [options]

Send one build request to the persistent Unity build daemon.

Options:
  --task TASK_ID
  --project-root PATH
  --worktree-path PATH
  --unity PATH
  --log-file PATH
  --skip-sync
  --sync-back
  --no-start
  --wait-timeout-sec SEC
  --result-grace-sec SEC
  --verbose
  --progress-interval-sec SEC
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task) TASK_ID="$2"; shift 2 ;;
        --project-root) PROJECT_ROOT="$2"; shift 2 ;;
        --worktree-path) WORKTREE_PATH="$2"; shift 2 ;;
        --unity) UNITY_EDITOR="$2"; shift 2 ;;
        --log-file) DAEMON_LOG_FILE="$2"; shift 2 ;;
        --skip-sync) SKIP_SYNC=1; shift ;;
        --sync-back) SYNC_BACK=1; shift ;;
        --no-start) START_IF_NEEDED=0; shift ;;
        --wait-timeout-sec) WAIT_TIMEOUT_SEC="$2"; shift 2 ;;
        --result-grace-sec) RESULT_GRACE_SEC="$2"; shift 2 ;;
        --verbose) VERBOSE=1; shift ;;
        --progress-interval-sec) PROGRESS_INTERVAL_SEC="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *)
            if [[ -z "$TASK_ID" ]]; then
                TASK_ID="$1"
                shift
            else
                die "Unknown argument: $1"
            fi
            ;;
    esac
done

[[ -n "$TASK_ID" ]] || die "Task id is required."
[[ "$RESULT_GRACE_SEC" =~ ^[0-9]+$ ]] || die "--result-grace-sec must be a non-negative integer."

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

if ! daemon_is_running; then
    [[ "$START_IF_NEEDED" -eq 1 ]] || die "Unity build daemon is not running."
    "${SCRIPT_DIR}/build_daemon_start.sh" \
        --project-root "$PROJECT_ROOT" \
        --worktree-path "$WORKTREE_PATH" \
        --unity "$UNITY_EDITOR" \
        --log-file "$DAEMON_LOG_FILE" \
        --skip-sync
elif [[ -f "$DAEMON_LOG_PATH_FILE" ]]; then
    RUNNING_DAEMON_LOG_FILE="$(<"$DAEMON_LOG_PATH_FILE")"
    if [[ -n "$RUNNING_DAEMON_LOG_FILE" && "$RUNNING_DAEMON_LOG_FILE" != "$DAEMON_LOG_FILE" ]]; then
        printf '[daemon] using running daemon logFile=%s\n' "$RUNNING_DAEMON_LOG_FILE"
        DAEMON_LOG_FILE="$RUNNING_DAEMON_LOG_FILE"
    fi
fi

REQUEST_ID="$(date +%s)-$$"
rm -f "$DAEMON_RESULT_FILE"
LOG_OFFSET=0
if [[ -f "$DAEMON_LOG_FILE" ]]; then
    LOG_OFFSET="$(stat -c %s "$DAEMON_LOG_FILE")"
fi
STREAM_OFFSET="$LOG_OFFSET"
BUILD_SUCCESS_SEEN_AT=""
python3 - "$DAEMON_REQUEST_FILE" "$REQUEST_ID" "$TASK_ID" <<'PY'
import json
import os
import sys

path, request_id, method = sys.argv[1:4]
tmp = path + ".tmp"
with open(tmp, "w", encoding="utf-8") as handle:
    json.dump({"id": request_id, "task_id": method}, handle)
os.replace(tmp, path)
PY

printf '[daemon] requested task=%s id=%s\n' "$TASK_ID" "$REQUEST_ID"

deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
started_at="$SECONDS"
last_progress_at="$SECONDS"
while [[ "$SECONDS" -lt "$deadline" ]]; do
    if [[ -f "$DAEMON_RESULT_FILE" ]]; then
        python3 - "$DAEMON_RESULT_FILE" "$REQUEST_ID" <<'PY'
import json
import sys

path, expected_id = sys.argv[1:3]
with open(path, "r", encoding="utf-8") as handle:
    result = json.load(handle)
print(json.dumps(result, indent=2, ensure_ascii=False))
if result.get("id") != expected_id:
    print(f"Unexpected result id: {result.get('id')} != {expected_id}", file=sys.stderr)
    sys.exit(2)
if not result.get("success"):
    sys.exit(1)
PY
        if [[ "$SYNC_BACK" -eq 1 ]]; then
            sync_back
        fi
        printf '[done] Unity build finished via daemon.\n'
        exit 0
    fi
    if ! daemon_is_running; then
        tail -n 160 "$DAEMON_LOG_FILE" 2>/dev/null || true
        die "Unity build daemon exited while building."
    fi
    if [[ -f "$DAEMON_LOG_FILE" ]]; then
        CURRENT_LOG_SIZE="$(stat -c %s "$DAEMON_LOG_FILE" 2>/dev/null || printf '0')"
        if [[ "$CURRENT_LOG_SIZE" -gt "$STREAM_OFFSET" ]]; then
            NEW_LOG="$(tail -c +"$((STREAM_OFFSET + 1))" "$DAEMON_LOG_FILE" 2>/dev/null || true)"
            STREAM_OFFSET="$CURRENT_LOG_SIZE"
            if [[ "$VERBOSE" -eq 1 && -n "$NEW_LOG" ]]; then
                printf '%s' "$NEW_LOG"
                [[ "$NEW_LOG" == *$'\n' ]] || printf '\n'
            fi
        else
            NEW_LOG=""
        fi

        if [[ -z "$BUILD_SUCCESS_SEEN_AT" ]] && grep -q 'Build Finished, Result: Success' <<<"$NEW_LOG"; then
            BUILD_SUCCESS_SEEN_AT="$SECONDS"
            printf '[daemon] BuildPipeline success detected in daemon log; waiting up to %ss for the daemon JSON result.\n' \
                "$RESULT_GRACE_SEC"
        fi
        if [[ -n "$BUILD_SUCCESS_SEEN_AT" && "$((SECONDS - BUILD_SUCCESS_SEEN_AT))" -ge "$RESULT_GRACE_SEC" ]]; then
            printf '[daemon] Unity did not return a JSON result within %ss; stopping daemon so the next build can restart cleanly.\n' \
                "$RESULT_GRACE_SEC"
            "${SCRIPT_DIR}/build_daemon_stop.sh" --project-root "$PROJECT_ROOT" --worktree-path "$WORKTREE_PATH" --force >/dev/null 2>&1 || true
            printf '[done] Unity build finished via daemon log fallback.\n'
            exit 0
        fi
        if grep -q 'Build Finished, Result: Failed' <<<"$NEW_LOG"; then
            tail -n 160 "$DAEMON_LOG_FILE" 2>/dev/null || true
            die "Unity BuildPipeline reported failure."
        fi
    fi
    if [[ "$VERBOSE" -eq 0 && "$((SECONDS - last_progress_at))" -ge "$PROGRESS_INTERVAL_SEC" ]]; then
        printf '[wait] Unity build still running elapsed=%ss timeout=%ss log=%s\n' \
            "$((SECONDS - started_at))" "$WAIT_TIMEOUT_SEC" "$DAEMON_LOG_FILE"
        last_progress_at="$SECONDS"
    fi
    sleep 1
done

tail -n 160 "$DAEMON_LOG_FILE" 2>/dev/null || true
die "Timed out waiting for build result after ${WAIT_TIMEOUT_SEC}s."
