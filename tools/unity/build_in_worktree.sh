#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
DEFAULT_WORKTREE_PATH="$(cd "${DEFAULT_PROJECT_ROOT}/.." && pwd -P)/finssimunity-build"
DEFAULT_UNITY_EDITOR="/home/fins/Unity/Hub/Editor/6000.3.20f1/Editor/Unity"
DEFAULT_TASK_ID="chase/three-headless"

PROJECT_ROOT="${FINSIM_UNITY_PROJECT_ROOT:-$DEFAULT_PROJECT_ROOT}"
WORKTREE_PATH="${FINSIM_UNITY_BUILD_WORKTREE:-$DEFAULT_WORKTREE_PATH}"
UNITY_EDITOR="${UNITY_EDITOR:-$DEFAULT_UNITY_EDITOR}"
TASK_ID="$DEFAULT_TASK_ID"
LOG_FILE=""
SYNC_ONLY=0
SKIP_SYNC=0
SYNC_BACK=0
USE_DAEMON=1
VERBOSE=0
UNITY_EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  tools/unity/build_in_worktree.sh [options] [-- extra-unity-args...]

Build the Unity project from a separate git worktree so the foreground Unity
Editor can stay open on the main project. By default this uses a persistent
Unity build daemon. Use --cold-start to run the old one-shot Editor build.

Default task:
  chase/three-headless

Options:
  --task ID       Catalog task id, for example chase/three-headless.
  --project-root PATH       Main Unity project. Defaults to this script's repo.
  --worktree-path PATH      Build worktree path. Defaults to ../finssimunity-build.
  --unity PATH              Unity Editor executable.
  --log-file PATH           Unity build log file. Defaults to /tmp/finssimunity-worktree-*.log.
  --sync-only               Create/update the build worktree and stop before Unity.
  --skip-sync               Do not rsync main project changes into the worktree.
  --sync-back               After a successful build, copy source files back to main.
                            Use only when you intentionally generated source assets
                            in the build worktree and the main Editor can reimport.
  --cold-start, --no-daemon Start a fresh Unity Editor for this build and quit after.
  --verbose                 Stream Unity build log while waiting.
  -h, --help                Show this help.

Environment overrides:
  FINSIM_UNITY_PROJECT_ROOT
  FINSIM_UNITY_BUILD_WORKTREE
  UNITY_EDITOR
  DISPLAY
  XAUTHORITY
  XDG_RUNTIME_DIR

Examples:
  tools/unity/build_in_worktree.sh

  tools/unity/build_in_worktree.sh \
    --task pose-control/new-server

  tools/unity/build_in_worktree.sh --sync-only
EOF
}

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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)
            [[ $# -ge 2 ]] || die "--task requires a value"
            TASK_ID="$2"
            shift 2
            ;;
        --project-root)
            [[ $# -ge 2 ]] || die "--project-root requires a value"
            PROJECT_ROOT="$2"
            shift 2
            ;;
        --worktree-path)
            [[ $# -ge 2 ]] || die "--worktree-path requires a value"
            WORKTREE_PATH="$2"
            shift 2
            ;;
        --unity)
            [[ $# -ge 2 ]] || die "--unity requires a value"
            UNITY_EDITOR="$2"
            shift 2
            ;;
        --log-file)
            [[ $# -ge 2 ]] || die "--log-file requires a value"
            LOG_FILE="$2"
            shift 2
            ;;
        --sync-only)
            SYNC_ONLY=1
            shift
            ;;
        --skip-sync)
            SKIP_SYNC=1
            shift
            ;;
        --sync-back)
            SYNC_BACK=1
            shift
            ;;
        --cold-start|--no-daemon)
            USE_DAEMON=0
            shift
            ;;
        --verbose)
            VERBOSE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            UNITY_EXTRA_ARGS+=("$@")
            break
            ;;
        *)
            die "Unknown argument: $1"
            ;;
    esac
done

require_cmd git
require_cmd rsync
require_cmd python3

PROJECT_ROOT="$(real_path "$PROJECT_ROOT")"
WORKTREE_PATH="$(real_path "$WORKTREE_PATH")"

[[ -d "$PROJECT_ROOT/.git" ]] || die "Not a git Unity project root: $PROJECT_ROOT"
[[ "$PROJECT_ROOT" != "$WORKTREE_PATH" ]] || die "worktree path must be different from project root"

if [[ -z "$LOG_FILE" ]]; then
    LOG_FILE="/tmp/finssimunity-worktree-$(sanitize_method_name "$TASK_ID").log"
fi

if [[ "$USE_DAEMON" -eq 1 && "$SYNC_ONLY" -eq 0 ]]; then
    if [[ "${#UNITY_EXTRA_ARGS[@]}" -gt 0 ]]; then
        printf '[daemon] extra Unity command-line args require --cold-start; falling back to one-shot build\n'
    else
        daemon_args=(
            --task "$TASK_ID"
            --project-root "$PROJECT_ROOT"
            --worktree-path "$WORKTREE_PATH"
            --unity "$UNITY_EDITOR"
            --log-file "$LOG_FILE"
        )
        if [[ "$SKIP_SYNC" -eq 1 ]]; then
            daemon_args+=(--skip-sync)
        fi
        if [[ "$SYNC_BACK" -eq 1 ]]; then
            daemon_args+=(--sync-back)
        fi
        if [[ "$VERBOSE" -eq 1 ]]; then
            daemon_args+=(--verbose)
        fi
        exec "$SCRIPT_DIR/build_daemon_request.sh" "${daemon_args[@]}"
    fi
fi

create_or_verify_worktree() {
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

fail_if_build_worktree_editor_is_open() {
    local matches
    matches="$(ps -eo pid=,args= | grep -F "$UNITY_EDITOR" | grep -F "$WORKTREE_PATH" | grep -v grep || true)"
    if [[ -n "$matches" ]]; then
        printf '%s\n' "$matches" >&2
        die "A Unity Editor process is already using the build worktree."
    fi
}

run_unity_build() {
    [[ -x "$UNITY_EDITOR" ]] || die "Unity Editor executable not found or not executable: $UNITY_EDITOR"
    mkdir -p "$(dirname "$LOG_FILE")"

    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

    printf '[unity] projectPath=%s\n' "$WORKTREE_PATH"
    printf '[unity] task=%s\n' "$TASK_ID"
    printf '[unity] logFile=%s\n' "$LOG_FILE"

    local tail_pid=""
    if [[ "$VERBOSE" -eq 1 ]]; then
        : >"$LOG_FILE"
        tail -n 0 -F "$LOG_FILE" &
        tail_pid="$!"
        trap '[[ -n "${tail_pid:-}" ]] && kill "$tail_pid" >/dev/null 2>&1 || true' EXIT
    fi

    "$UNITY_EDITOR" \
        -batchmode \
        -quit \
        -projectPath "$WORKTREE_PATH" \
        -executeMethod FinsSimTaskBuild.BuildFromCommandLine \
        -finsSimTask "$TASK_ID" \
        -logFile "$LOG_FILE" \
        "${UNITY_EXTRA_ARGS[@]}"

    if [[ -n "$tail_pid" ]]; then
        kill "$tail_pid" >/dev/null 2>&1 || true
        wait "$tail_pid" 2>/dev/null || true
        trap - EXIT
    fi
}

create_or_verify_worktree

if [[ "$SKIP_SYNC" -eq 0 ]]; then
    sync_forward
else
    printf '[sync] skipped by --skip-sync\n'
fi

if [[ "$SYNC_ONLY" -eq 1 ]]; then
    printf '[done] synced build worktree: %s\n' "$WORKTREE_PATH"
    exit 0
fi

fail_if_build_worktree_editor_is_open
run_unity_build

if [[ "$SYNC_BACK" -eq 1 ]]; then
    sync_back
fi

printf '[done] Unity build finished via worktree.\n'
