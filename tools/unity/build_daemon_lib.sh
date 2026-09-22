#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
DEFAULT_WORKTREE_PATH="$(cd "${DEFAULT_PROJECT_ROOT}/.." && pwd -P)/finssimunity-build"
DEFAULT_UNITY_EDITOR="/home/fins/Unity/Hub/Editor/6000.3.20f1/Editor/Unity"

PROJECT_ROOT="${FINSIM_UNITY_PROJECT_ROOT:-$DEFAULT_PROJECT_ROOT}"
WORKTREE_PATH="${FINSIM_UNITY_BUILD_WORKTREE:-$DEFAULT_WORKTREE_PATH}"
UNITY_EDITOR="${UNITY_EDITOR:-$DEFAULT_UNITY_EDITOR}"
DAEMON_STATE_DIR="${FINSIM_UNITY_BUILD_DAEMON_DIR:-/tmp/finsim-unity-build-daemon}"
DAEMON_REQUEST_FILE="${DAEMON_STATE_DIR}/request.json"
DAEMON_RESULT_FILE="${DAEMON_STATE_DIR}/result.json"
DAEMON_READY_FILE="${DAEMON_STATE_DIR}/ready"
DAEMON_PID_FILE="${DAEMON_STATE_DIR}/unity.pid"
DAEMON_LOG_PATH_FILE="${DAEMON_STATE_DIR}/log_path"
DAEMON_LOG_FILE="${FINSIM_UNITY_BUILD_DAEMON_LOG:-/tmp/finsim-unity-build-daemon.log}"

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

sanitize_method_name() {
    printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_' | sed 's/__*/_/g'
}

real_path() {
    python3 -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' "$1"
}

normalize_paths() {
    PROJECT_ROOT="$(real_path "$PROJECT_ROOT")"
    WORKTREE_PATH="$(real_path "$WORKTREE_PATH")"
}

create_or_verify_worktree() {
    [[ -d "$PROJECT_ROOT/.git" ]] || die "Not a git Unity project root: $PROJECT_ROOT"
    [[ "$PROJECT_ROOT" != "$WORKTREE_PATH" ]] || die "worktree path must be different from project root"

    if [[ ! -e "$WORKTREE_PATH" ]]; then
        mkdir -p "$(dirname "$WORKTREE_PATH")"
        printf '[worktree] creating %s from %s\n' "$WORKTREE_PATH" "$PROJECT_ROOT"
        git -C "$PROJECT_ROOT" worktree add --detach "$WORKTREE_PATH" HEAD
    fi

    [[ -e "$WORKTREE_PATH/.git" ]] || die "Path exists but is not a git worktree: $WORKTREE_PATH"

    local main_common build_common
    main_common="$(real_path "$(git -C "$PROJECT_ROOT" rev-parse --git-common-dir)")"
    build_common="$(real_path "$(git -C "$WORKTREE_PATH" rev-parse --git-common-dir)")"
    [[ "$main_common" == "$build_common" ]] || die "worktree does not belong to the same git repo: $WORKTREE_PATH"
}

sync_forward() {
    printf '[sync] main -> build worktree\n'
    rsync -a --delete --human-readable --info=stats2 \
        --filter='P /.git' \
        --exclude='/.git/' \
        --exclude='/.git' \
        --exclude='/Library/' \
        --exclude='/Temp/' \
        --exclude='/Obj/' \
        --exclude='/Logs/' \
        --exclude='/UserSettings/' \
        --exclude='/Build/' \
        --exclude='/Builds/' \
        --exclude='/MemoryCaptures/' \
        --exclude='/.vs/' \
        --exclude='/ExportedObj/' \
        --exclude='/mono_crash.mem.*.blob' \
        --exclude='mono_crash.mem.*.blob' \
        --exclude='/*.csproj' \
        --exclude='/*.sln' \
        "$PROJECT_ROOT/" "$WORKTREE_PATH/"

    if command -v git-lfs >/dev/null 2>&1; then
        git -C "$WORKTREE_PATH" lfs checkout >/dev/null 2>&1 || true
    fi
}

sync_back() {
    printf '[sync] build worktree -> main project (source files only)\n'
    printf '[sync] WARNING: this can trigger reimport in the foreground Unity Editor.\n'
    for dir in Assets Packages ProjectSettings; do
        [[ -d "$WORKTREE_PATH/$dir" ]] || continue
        rsync -a --human-readable --info=stats2 \
            "$WORKTREE_PATH/$dir/" "$PROJECT_ROOT/$dir/"
    done

    for file in AGENTS.md README.md .gitignore .gitattributes; do
        if [[ -f "$WORKTREE_PATH/$file" ]]; then
            rsync -a "$WORKTREE_PATH/$file" "$PROJECT_ROOT/$file"
        fi
    done

    if [[ -d "$WORKTREE_PATH/tools" ]]; then
        rsync -a --delete "$WORKTREE_PATH/tools/" "$PROJECT_ROOT/tools/"
    fi
}

unity_process_for_worktree() {
    local expected_editor
    expected_editor="$(real_path "$UNITY_EDITOR")"

    ps -eo pid=,args= \
        | grep -F "$WORKTREE_PATH" \
        | grep -E -- '-project[Pp]ath[ =]' \
        | grep -F -- '-batchmode' \
        | grep -F -- '-executeMethod FinsSimBuildDaemon.Start' \
        | grep -v grep \
        | while read -r pid _args; do
            local actual_editor
            actual_editor="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
            if [[ "$actual_editor" == "$expected_editor" ]]; then
                printf '%s\n' "$pid"
                break
            fi
        done
}

daemon_is_running() {
    local pid
    pid="$(unity_process_for_worktree || true)"
    [[ -n "$pid" ]] || return 1
    kill -0 "$pid" >/dev/null 2>&1
}

init_daemon_env() {
    mkdir -p "$DAEMON_STATE_DIR" "$(dirname "$DAEMON_LOG_FILE")"
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    export FINSIM_UNITY_BUILD_REQUEST="$DAEMON_REQUEST_FILE"
    export FINSIM_UNITY_BUILD_RESULT="$DAEMON_RESULT_FILE"
    export FINSIM_UNITY_BUILD_READY="$DAEMON_READY_FILE"
}
