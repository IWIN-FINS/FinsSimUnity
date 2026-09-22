#!/usr/bin/env bash
# Create a history-free public release from a private release-candidate branch.
set -Eeuo pipefail

program_name="$(basename "$0")"

die() {
    printf '%s: %s\n' "$program_name" "$*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage:
  scripts/publish_public_release.sh --version VERSION [options]

Creates a root commit from release-candidate, records it locally as
release/VERSION, creates an annotated tag (vVERSION by default), pushes the
private release branch to origin-private, and force-with-lease updates the
public origin/main branch plus its tag.

Options:
  --version VERSION       Required release version, e.g. 0.1.0.
  --tag TAG               Annotated tag name (default: vVERSION).
  --source BRANCH         Source branch (default: release-candidate).
  --exclude PATH          Remove a tracked path or Git pathspec; repeatable.
  --exclude-file FILE     Read one exclusion rule per line; repeatable.
  --dry-run               Validate and show the generated release tree only.
  --amend-release         Explicitly reissue an existing VERSION after an
                          approved corrective change. Replaces release/VERSION,
                          public origin/main, and its tag with lease protection.
  -h, --help              Show this help text.

Exclusion rules run only in an isolated temporary worktree. They never modify
the source branch. Blank lines and lines beginning with # in exclusion files
are ignored. The candidate tree must be committed first; uncommitted changes
are intentionally not included in a release.
EOF
}

version=""
tag_name=""
source_branch=""
dry_run=false
amend_release=false
declare -a exclusion_rules=()
declare -a exclusion_files=()

while (($#)); do
    case "$1" in
        --version) version="${2:-}"; shift 2 ;;
        --tag) tag_name="${2:-}"; shift 2 ;;
        --source) source_branch="${2:-}"; shift 2 ;;
        --exclude) exclusion_rules+=("${2:-}"); shift 2 ;;
        --exclude-file) exclusion_files+=("${2:-}"); shift 2 ;;
        --dry-run) dry_run=true; shift ;;
        --amend-release) amend_release=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown argument: $1" ;;
    esac
done

[[ -n "$version" ]] || die "--version is required"
[[ "$version" =~ ^[0-9A-Za-z][0-9A-Za-z._-]*$ ]] || die "invalid version: $version"
source_branch="${source_branch:-release-candidate}"
tag_name="${tag_name:-v$version}"
release_branch="release/$version"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git -C "$script_dir/.." rev-parse --show-toplevel 2>/dev/null)" || die "script must reside in a Git repository"
cd "$repo_root"
git remote get-url origin >/dev/null || die "missing public remote: origin"
git remote get-url origin-private >/dev/null || die "missing private remote: origin-private"
git rev-parse --verify --quiet "$source_branch^{commit}" >/dev/null || die "source branch not found: $source_branch"

for exclusion_file in "${exclusion_files[@]}"; do
    [[ -f "$exclusion_file" ]] || die "exclusion file not found: $exclusion_file"
    while IFS= read -r rule || [[ -n "$rule" ]]; do
        [[ -z "$rule" || "$rule" == \#* ]] && continue
        exclusion_rules+=("$rule")
    done < "$exclusion_file"
done

for rule in "${exclusion_rules[@]}"; do
    [[ -n "$rule" ]] || die "empty exclusion rule"
    [[ "$rule" != /* && "$rule" != .. && "$rule" != ../* && "$rule" != */../* ]] || die "unsafe exclusion rule: $rule"
done

temporary_worktree="$(mktemp -d "${TMPDIR:-/tmp}/$(basename "$repo_root")-release-${version}-XXXXXX")"
worktree_created=false
cleanup() {
    if "$worktree_created"; then
        git -C "$repo_root" worktree remove --force "$temporary_worktree" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

git worktree add --detach "$temporary_worktree" "$source_branch" >/dev/null
worktree_created=true

for rule in "${exclusion_rules[@]}"; do
    git -C "$temporary_worktree" rm -r --ignore-unmatch -- "$rule"
done

release_tree="$(git -C "$temporary_worktree" write-tree)"
printf 'Source:       %s (%s)\n' "$source_branch" "$(git rev-parse "$source_branch")"
printf 'Release tree: %s\n' "$release_tree"
printf 'Exclusions:   %d\n' "${#exclusion_rules[@]}"

if "$dry_run"; then
    printf 'Dry run only: no branch, tag, or remote ref was changed.\n'
    exit 0
fi

if git show-ref --verify --quiet "refs/heads/$release_branch"; then
    release_commit="$(git rev-parse "$release_branch")"
    [[ -z "$(git show -s --format=%P "$release_commit")" ]] || die "local $release_branch is not an orphan release commit"
    if [[ "$(git rev-parse "$release_commit^{tree}")" != "$release_tree" ]]; then
        "$amend_release" || die "local $release_branch already points to a different release tree; use a new version (or an approved --amend-release)"
        release_commit="$(git commit-tree "$release_tree" -m "release: $(basename "$repo_root") $version (corrected)")"
        git branch -f "$release_branch" "$release_commit"
    fi
else
    release_commit="$(git commit-tree "$release_tree" -m "release: $(basename "$repo_root") $version")"
    git branch "$release_branch" "$release_commit"
fi

if git rev-parse --verify --quiet "refs/tags/$tag_name" >/dev/null; then
    if [[ "$(git rev-list -n 1 "$tag_name")" != "$release_commit" ]]; then
        "$amend_release" || die "local tag $tag_name points to a different release; tags are immutable"
        git tag -fa "$tag_name" "$release_commit" -m "Release $version (corrected)"
    fi
else
    git tag -a "$tag_name" "$release_commit" -m "Release $version"
fi

remote_tag="$(git ls-remote --tags origin "refs/tags/$tag_name" | awk 'NR == 1 { print $1 }')"
local_tag="$(git rev-parse "refs/tags/$tag_name")"
if [[ -n "$remote_tag" && "$remote_tag" != "$local_tag" ]]; then
    "$amend_release" || die "public tag $tag_name already exists with different content"
fi

private_release="$(git ls-remote --heads origin-private "refs/heads/$release_branch" | awk 'NR == 1 { print $1 }')"
if "$amend_release"; then
    git push --force-with-lease="refs/heads/$release_branch:$private_release" origin-private \
        "$release_commit:refs/heads/$release_branch"
else
    git push origin-private "refs/heads/$release_branch:refs/heads/$release_branch"
fi

remote_main="$(git ls-remote --heads origin refs/heads/main | awk 'NR == 1 { print $1 }')"
public_refs=("$release_commit:refs/heads/main")
if [[ -z "$remote_tag" ]]; then
    public_refs+=("refs/tags/$tag_name")
elif "$amend_release" && [[ "$remote_tag" != "$local_tag" ]]; then
    public_refs+=("+refs/tags/$tag_name")
fi
git push --force-with-lease="refs/heads/main:$remote_main" origin "${public_refs[@]}"

printf 'Published %s (%s) to origin/main with tag %s.\n' "$release_branch" "$release_commit" "$tag_name"
