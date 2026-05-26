#!/usr/bin/env bash
# Sync between the laptop (canonical source of truth for code/docs) and the NUC.
#
#   sync.sh push       laptop code/docs  -> NUC      (laptop is authoritative)
#   sync.sh pull        NUC run artifacts -> laptop   (bags/sidecars flow back)
#   sync.sh push-dry / pull-dry            same, but rsync --dry-run (preview)
#
# Transport is plain rsync over **Tailscale SSH**, which authenticates the
# agilex@nuc connection passwordless — no `expect`/password needed (that was the
# old stopgap). If Tailscale is ever down, fall back to the `expect` pattern in
# CLAUDE.md.
#
# Direction of truth:
#   - code/docs live in git and are edited on the laptop  -> only ever PUSHed.
#   - run artifacts (rosbags, sidecars) are generated on the NUC under
#     "Experiment Data/" -> only ever PULLed (gitignored; too large for git).
#
# This is the interim mechanism. The intended long-term split is: git (a shared
# remote both machines push/pull) for code, and this tool for the large
# artifacts that don't belong in git.
set -euo pipefail

NUC="${LIMO_HOST:-agilex@agilex-nuc12wski7}"
LAPTOP_ROOT="${LIMO_LAPTOP_ROOT:-/Users/hyeon-yongjeong/code/H-infinity}"
NUC_ROOT="${LIMO_NUC_ROOT:-/home/agilex/H-infinity}"
ARTIFACTS="Experiment Data"
SSH_E=(-e "ssh -o BatchMode=yes -o ConnectTimeout=20")

CODE_EXCLUDES=(
  --exclude=.git --exclude=.DS_Store --exclude=__pycache__ --exclude='*.pyc'
  --exclude=.specstory --exclude=.vscode --exclude=.claude
  --exclude=build --exclude=install --exclude=log --exclude=.pytest_cache
  --exclude="$ARTIFACTS/"            # never push artifacts up
)

usage() { echo "usage: $0 {push|pull|push-dry|pull-dry}"; exit 2; }

DRY=""
cmd="${1:-}"
case "$cmd" in
  push-dry) cmd=push; DRY="--dry-run" ;;
  pull-dry) cmd=pull; DRY="--dry-run" ;;
  push|pull) ;;
  *) usage ;;
esac

case "$cmd" in
  push)
    echo "[sync] push  laptop -> $NUC   (code/docs; laptop canonical) $DRY"
    rsync -avz $DRY "${SSH_E[@]}" "${CODE_EXCLUDES[@]}" \
      "$LAPTOP_ROOT/" "$NUC:$NUC_ROOT/"
    # NB: no --delete (additive) so NUC-unique files survive; flip on once the
    # laptop tree is trusted to be a complete mirror.
    ;;
  pull)
    echo "[sync] pull  $NUC -> laptop   (run artifacts) $DRY"
    mkdir -p "$LAPTOP_ROOT/$ARTIFACTS"
    # rsync word-splits the *remote* path; "Experiment Data" has a space and
    # macOS rsync lacks --protect-args/-s, so escape the space in the remote arg
    # (the local path is a single quoted arg and needs no escaping).
    rsync -avz $DRY "${SSH_E[@]}" \
      "$NUC:$NUC_ROOT/${ARTIFACTS// /\\ }/" "$LAPTOP_ROOT/$ARTIFACTS/"
    ;;
esac
echo "[sync] done."
