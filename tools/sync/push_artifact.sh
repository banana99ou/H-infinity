#!/usr/bin/env bash
# Push ONE finished bag directory off the robot to the artifact archives,
# rate-limited and low-priority so it never disturbs an active run. Runs ON THE
# NUC; invoked (detached) by the sequencer after each leg's bag + sidecar are
# written.
#
# Targets come from the environment (the sequencer sets these from the
# experiment.yaml `artifact_sync` block):
#   ARTIFACT_MAC_TARGET    user@host:"/path/Experiment Data/"   (priority; first)
#   ARTIFACT_NAS_TARGET    user@host:"/path/Experiment Data/"
#   ARTIFACT_BWLIMIT_KBPS  rsync --bwlimit value (0/empty = unlimited)
#
# .git is never transferred (each archive keeps its own independent history).
# A failed/unreachable target is logged and skipped — copying out a bag must
# NEVER break the batch. macOS targets need Remote Login enabled + the NUC key
# authorized.
set -uo pipefail

BAG="${1:?usage: push_artifact.sh <bag_dir>}"
[ -d "$BAG" ] || { echo "[push_artifact] no such bag dir: $BAG" >&2; exit 0; }

BW="${ARTIFACT_BWLIMIT_KBPS:-0}"
BWOPT=""; { [ -n "$BW" ] && [ "$BW" != "0" ]; } && BWOPT="--bwlimit=$BW"

NICE="nice -n 19"
IONICE=""; command -v ionice >/dev/null 2>&1 && IONICE="ionice -c3"
base="$(basename "$BAG")"

push_one() {  # $1 = label, $2 = target "user@host:/path/"
  local label="$1" target="$2"
  [ -n "$target" ] || { echo "[push_artifact] $label: no target set, skip"; return 0; }
  echo "[push_artifact] -> $label : ${target}${base}"
  # -s/--protect-args handles the space in "Experiment Data" (Linux rsync).
  if $NICE $IONICE rsync -azs $BWOPT \
       -e "ssh -o BatchMode=yes -o ConnectTimeout=20" --exclude='.git' \
       "$BAG" "$target"; then
    echo "[push_artifact] $label OK: $base"
  else
    echo "[push_artifact] $label FAILED (skipped, non-fatal): $base" >&2
  fi
}

push_one "macbook" "${ARTIFACT_MAC_TARGET:-}"   # priority: Mac first
push_one "nas"     "${ARTIFACT_NAS_TARGET:-}"

# The robot keeps its own local git history of the artifact too.
LR="${ARTIFACT_LOCAL_REPO:-}"
if [ -n "$LR" ] && [ -d "$LR/.git" ]; then
  git -C "$LR" add -A
  if ! git -C "$LR" diff --cached --quiet; then
    git -C "$LR" commit -q -m "archive $base $(date -u +%FT%TZ)" \
      && echo "[push_artifact] local repo committed: $base"
  fi
fi
echo "[push_artifact] done: $base"
