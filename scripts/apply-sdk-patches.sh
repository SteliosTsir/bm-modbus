#!/bin/bash
# Applies patches/*.patch to the BareMetal submodule (sdk/). Safe to re-run:
# patches that are already applied are skipped.
set -e
cd "$(dirname "$0")/.."
SDK=sdk
[ -e "$SDK/.git" ] || { echo "sdk/ submodule missing: git submodule update --init"; exit 1; }
for p in patches/*.patch; do
	[ -e "$p" ] || exit 0
	if git -C "$SDK" apply --reverse --check "$PWD/$p" 2>/dev/null; then
		echo "  [PATCH] $(basename "$p"): already applied"
	elif git -C "$SDK" apply --check "$PWD/$p" 2>/dev/null; then
		git -C "$SDK" apply "$PWD/$p"
		echo "  [PATCH] $(basename "$p"): applied"
	else
		echo "  [PATCH] $(basename "$p"): does NOT apply to this SDK commit" >&2
		exit 1
	fi
done
