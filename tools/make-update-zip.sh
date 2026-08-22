#!/usr/bin/env bash
#
# Build an "update only" zip containing just the files that changed between two
# releases, with paths relative to the aircraft folder root so it can be
# extracted straight over an existing install.
#
# Usage:
#   tools/make-update-zip.sh <from-ref> <to-ref> [output.zip]
#
# Examples:
#   tools/make-update-zip.sh v1.0.0 v1.1.0
#   tools/make-update-zip.sh v1.1.0 HEAD /tmp/preview.zip
#
# The script also reports any files that were DELETED between the two refs.
# A zip cannot express a deletion, so those have to be removed by hand - they
# are listed in DELETED-FILES.txt inside the zip when there are any.

set -euo pipefail

FROM="${1:-}"
TO="${2:-HEAD}"

if [ -z "$FROM" ]; then
    echo "usage: $0 <from-ref> <to-ref> [output.zip]" >&2
    echo "" >&2
    echo "available release tags:" >&2
    git tag -l 'v*' | sort -V | sed 's/^/  /' >&2
    exit 1
fi

cd "$(git rev-parse --show-toplevel)"

FROM_NAME="$(git rev-parse --abbrev-ref "$FROM" 2>/dev/null || echo "$FROM")"
TO_NAME="$(git rev-parse --abbrev-ref "$TO" 2>/dev/null || echo "$TO")"
OUT="${3:-B747-400_DJMod_${FROM_NAME}_to_${TO_NAME}_update.zip}"

# Added, Copied, Modified, Renamed (new name) - i.e. everything that must ship.
#
# releases/ is excluded: it holds previously published update packages, which
# are distribution metadata rather than aircraft files. Without this an update
# zip ends up containing the previous update zip.
mapfile -t CHANGED < <(git diff --name-only --diff-filter=ACMR "$FROM" "$TO" -- . ':(exclude)releases/*')
mapfile -t DELETED < <(git diff --name-only --diff-filter=D "$FROM" "$TO" -- . ':(exclude)releases/*')

if [ "${#CHANGED[@]}" -eq 0 ] && [ "${#DELETED[@]}" -eq 0 ]; then
    echo "No differences between $FROM_NAME and $TO_NAME - nothing to package."
    exit 0
fi

echo "Update package: $FROM_NAME -> $TO_NAME"
echo "  ${#CHANGED[@]} changed file(s), ${#DELETED[@]} deleted file(s)"
echo ""

rm -f "$OUT"

if [ "${#CHANGED[@]}" -gt 0 ]; then
    printf '%s\n' "${CHANGED[@]}" | sed 's/^/  + /'
    git archive --format=zip -o "$OUT" "$TO" -- "${CHANGED[@]}"
fi

if [ "${#DELETED[@]}" -gt 0 ]; then
    echo ""
    echo "  Files DELETED since $FROM_NAME (remove these by hand):"
    printf '%s\n' "${DELETED[@]}" | sed 's/^/  - /'
    TMPDIR_D="$(mktemp -d)"
    {
        echo "These files were removed between $FROM_NAME and $TO_NAME."
        echo "Delete them from your aircraft folder after extracting this update."
        echo ""
        printf '%s\n' "${DELETED[@]}"
    } > "$TMPDIR_D/DELETED-FILES.txt"
    ( cd "$TMPDIR_D" && zip -q "$OLDPWD/$OUT" DELETED-FILES.txt )
    rm -rf "$TMPDIR_D"
fi

echo ""
echo "Wrote $OUT ($(du -h "$OUT" | cut -f1))"
echo "Extract it over your existing 747-400 aircraft folder, keeping directory structure."
