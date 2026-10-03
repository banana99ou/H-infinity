#!/usr/bin/env bash
# Sync between the laptop (canonical source of truth for code/docs) and the NUC.
#
#   sync.sh push         laptop code/docs -> relay -> NUC, built + stack restarted
#   sync.sh push-direct  laptop code/docs -> NUC now (copy only: no build/restart)
#   sync.sh pull         NUC run artifacts -> laptop   (bags/sidecars flow back)
#   sync.sh push-dry / pull-dry            same, but rsync --dry-run (preview)
#   sync.sh relay-install / relay-log      set up / inspect the relay deployer
#
# `push` is store-and-forward: it stages the laptop tree on the always-on relay
# (work-fmcl, ~/hinf-relay/stage) and the relay's cron job (relay_deploy.sh)
# delivers it the moment the robot is online and idle — copy (overwritten robot
# files backed up), colcon build, restart of the limo-battle stack. So a push
# made while the robot is off still lands, and the robot runs what was pushed.
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
RELAY="${LIMO_RELAY:-work-fmcl}"
SSH_E=(-e "ssh -o BatchMode=yes -o ConnectTimeout=20")

# One exclude list for every code push (laptop->NUC, laptop->relay, relay->NUC).
# It also keeps "Experiment Data/" out: artifacts are never pushed up.
CODE_EXCLUDES=(--exclude-from="$LAPTOP_ROOT/tools/sync/code_excludes.txt")

ART_PATH="$LAPTOP_ROOT/$ARTIFACTS"

# The artifact folder is its OWN local-only git repo (separate from the code
# repo, which gitignores it). It tracks run-artifact history without git-LFS and
# without bloating the code repo, and never gets a remote / never pushes (a
# pre-push hook blocks it). See `init-artifacts`.
_artifacts_commit() {  # $1 = commit message; no-op unless the repo has changes
  [ -d "$ART_PATH/.git" ] || return 0
  git -C "$ART_PATH" add -A
  if ! git -C "$ART_PATH" diff --cached --quiet; then
    git -C "$ART_PATH" commit -q -m "$1"
    echo "[sync] artifact repo: committed '$1'"
  fi
}

usage() { echo "usage: $0 {push|push-direct|pull|push-dry|pull-dry|relay-install|relay-log|init-artifacts}"; exit 2; }

DRY=""
cmd="${1:-}"
case "$cmd" in
  push-dry) cmd=push; DRY="--dry-run" ;;
  pull-dry) cmd=pull; DRY="--dry-run" ;;
  push|push-direct|pull|relay-install|relay-log|init-artifacts) ;;
  *) usage ;;
esac

case "$cmd" in
  push)
    echo "[sync] push  laptop -> $RELAY -> $NUC   (code/docs; laptop canonical) $DRY"
    if [ -n "$DRY" ]; then
      rsync -avz --delete --dry-run "${SSH_E[@]}" "${CODE_EXCLUDES[@]}" \
        "$LAPTOP_ROOT/" "$RELAY:hinf-relay/stage/"
    else
      # Withdraw the old stamp under the relay's lock first: the relay never
      # deploys without a stamp, so it cannot ship a half-copied stage.
      ssh -o BatchMode=yes -o ConnectTimeout=20 "$RELAY" \
        'mkdir -p ~/hinf-relay && flock -w 900 ~/hinf-relay/.lock rm -f ~/hinf-relay/stage.stamp'
      # The stage mirrors the laptop exactly (--delete is safe: relay-only dir).
      rsync -az --delete "${SSH_E[@]}" "${CODE_EXCLUDES[@]}" \
        "$LAPTOP_ROOT/" "$RELAY:hinf-relay/stage/"
      dirty=""; git -C "$LAPTOP_ROOT" diff --quiet HEAD -- 2>/dev/null || dirty="+dirty"
      stamp="$(date -u +%FT%TZ) $(git -C "$LAPTOP_ROOT" rev-parse --short HEAD)$dirty"
      printf '%s\n' "$stamp" | ssh -o BatchMode=yes "$RELAY" \
        'cat > ~/hinf-relay/stage.stamp.tmp && mv ~/hinf-relay/stage.stamp.tmp ~/hinf-relay/stage.stamp'
      # Kick a deploy attempt now instead of waiting for the next cron minute.
      ssh -o BatchMode=yes "$RELAY" \
        'nohup bash ~/hinf-relay/stage/tools/sync/relay_deploy.sh >/dev/null 2>&1 </dev/null &'
      echo "[sync] staged '$stamp' on $RELAY; it deploys when the robot is online + idle."
      echo "[sync] watch: tools/sync/sync.sh relay-log"
    fi
    ;;
  push-direct)
    echo "[sync] push-direct  laptop -> $NUC   (copy only: no build, no restart)"
    rsync -avz "${SSH_E[@]}" "${CODE_EXCLUDES[@]}" \
      "$LAPTOP_ROOT/" "$NUC:$NUC_ROOT/"
    # NB: no --delete (additive) so NUC-unique files survive.
    ;;
  relay-install)
    # cron (not a systemd user unit) so it runs without a login session and
    # without `loginctl enable-linger` (sudo). Idempotent: replaces its own line.
    echo "[sync] installing the relay deployer cron job on $RELAY"
    ssh -o BatchMode=yes "$RELAY" 'mkdir -p ~/hinf-relay && { crontab -l 2>/dev/null | grep -v "# hinf-relay$"; echo "* * * * * bash \$HOME/hinf-relay/stage/tools/sync/relay_deploy.sh # hinf-relay"; } | crontab - && crontab -l | grep hinf-relay'
    ;;
  relay-log)
    ssh -o BatchMode=yes "$RELAY" 'echo "stage: $(cat ~/hinf-relay/stage.stamp 2>/dev/null || echo none)"; echo "state: $(cat ~/hinf-relay/state 2>/dev/null)"; tail -n 25 ~/hinf-relay/relay.log 2>/dev/null'
    ;;
  pull)
    echo "[sync] pull  $NUC -> laptop   (run artifacts) $DRY"
    mkdir -p "$LAPTOP_ROOT/$ARTIFACTS"
    # rsync word-splits the *remote* path; "Experiment Data" has a space and
    # macOS rsync lacks --protect-args/-s, so escape the space in the remote arg
    # (the local path is a single quoted arg and needs no escaping).
    rsync -avz $DRY "${SSH_E[@]}" \
      "$NUC:$NUC_ROOT/${ARTIFACTS// /\\ }/" "$LAPTOP_ROOT/$ARTIFACTS/"
    # Track artifact history in the folder's own local-only repo (if init'd).
    [ -n "$DRY" ] || _artifacts_commit "pull $(date -u +%FT%TZ): artifacts from NUC"
    ;;
  init-artifacts)
    echo "[sync] init local-only artifact archive at: $ART_PATH"
    mkdir -p "$ART_PATH"
    if [ -d "$ART_PATH/.git" ]; then
      echo "[sync] artifact repo already initialized."
    else
      git -C "$ART_PATH" init -q
      hook="$ART_PATH/.git/hooks/pre-push"
      printf '%s\n' '#!/bin/sh' \
        'echo "Refusing to push: local-only run-artifact archive (never upload run data off-site)." >&2' \
        'exit 1' > "$hook"
      chmod +x "$hook"
      if [ ! -f "$ART_PATH/README.md" ]; then
        cat > "$ART_PATH/README.md" <<'EOF'
# Experiment Data — local-only artifact archive

rosbags + per-leg sidecar JSON from the robot. This is its OWN git repo,
separate from the H-infinity code repo (which gitignores this folder). It tracks
run-artifact history locally WITHOUT git-LFS and without bloating the code repo,
and it NEVER gets a remote / is NEVER pushed (a pre-push hook blocks it).

Populated by `tools/sync/sync.sh pull` (NUC -> laptop), which auto-commits new
artifacts here. Do not add a git remote.
EOF
      fi
      git -C "$ART_PATH" add -A
      git -C "$ART_PATH" commit -q -m "init local-only artifact archive"
      echo "[sync] initialized: no remote, push blocked by pre-push hook."
    fi
    ;;
esac
echo "[sync] done."
