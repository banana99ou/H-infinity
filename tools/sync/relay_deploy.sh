#!/usr/bin/env bash
# Store-and-forward code deployer. Runs every minute from cron on the always-on
# relay (work-fmcl); the laptop stays the source of truth:
#
#   laptop  -- sync.sh push -->  relay ~/hinf-relay/stage/ + stage.stamp
#   relay   -- this script  -->  robot ~/H-infinity, build, restart the stack
#
# A deploy happens when the robot is reachable, its deployed stamp differs from
# stage.stamp, and no run is live (robot_side.sh busy). Every robot file the
# deploy overwrites is kept under ~/H-infinity-deploy-backups/<UTC time>/ on the
# robot — that is the "stash" of robot-side edits. Robot-only files (venue
# JSONs, discord.env, run artifacts) are never deleted: no --delete.
#
# Install / inspect from the laptop: sync.sh relay-install | relay-log.
# Log: ~/hinf-relay/relay.log (steady states are logged once, not per minute).
set -o pipefail

RELAY="$HOME/hinf-relay"
STAGE="$RELAY/stage"
STAMP_FILE="$RELAY/stage.stamp"
LOG="$RELAY/relay.log"
ROBOT="${LIMO_HOST:-agilex@agilex-nuc12wski7}"
ROBOT_ROOT="/home/agilex/H-infinity"
SIDE="$STAGE/tools/sync/robot_side.sh"
SSH_OPTS="-o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new"
# Runtime work owed to the robot: set when runtime files land, cleared only after
# a successful build + restart, so a deferred restart or a failed build is
# retried even though the next rsync copies nothing.
PENDING="$RELAY/pending_runtime"
PENDING_VENDOR="$RELAY/pending_vendor"

log()  { echo "$(date -u +%FT%TZ) $*" >> "$LOG"; }
# Log only when the steady-state reason changes (an offline robot must not
# write a line a minute).
note() {
  [ "$(cat "$RELAY/state" 2>/dev/null)" = "$1" ] && return
  echo "$1" > "$RELAY/state"; log "$1"
}
# ssh joins its arguments into ONE remote command line and the remote shell
# re-splits it, so each argument is %q-quoted (the stamp contains a space).
side() {
  local t=$1; shift
  timeout "$t" ssh $SSH_OPTS "$ROBOT" "bash -s -- $(printf '%q ' "$@")" < "$SIDE"
}

mkdir -p "$RELAY"
exec 9>"$RELAY/.lock"
flock -n 9 || exit 0          # a deploy, or a laptop push, is in progress

[ -s "$STAMP_FILE" ] || { note "waiting: nothing staged (or a laptop push is mid-way)"; exit 0; }
stamp=$(cat "$STAMP_FILE")

probe=$(timeout 30 ssh $SSH_OPTS "$ROBOT" true 2>&1); rc=$?
if [ $rc -ne 0 ]; then
  case "$probe" in
    *login.tailscale.com*) note "blocked: Tailscale SSH wants a browser check (ACL must 'accept' tag:relay)" ;;
    *) note "waiting: robot unreachable" ;;
  esac
  exit 0
fi

deployed=$(side 30 stamp 2>/dev/null)
if [ "$deployed" = "$stamp" ] && [ ! -e "$PENDING" ]; then note "in sync: $stamp"; exit 0; fi
if [ "$(cat "$RELAY/failed.stamp" 2>/dev/null)" = "$stamp" ]; then
  note "holding: deploy failed for $stamp — push a fix"; exit 0
fi
# At most ONE full deploy (and stack restart) per staged stamp. If the robot
# disagrees after we already delivered this stamp, something is wrong with the
# stamp bookkeeping: hold instead of restarting the robot every minute.
if [ "$(cat "$RELAY/delivered.stamp" 2>/dev/null)" = "$stamp" ] && [ ! -e "$PENDING" ]; then
  note "holding: $stamp was already delivered but the robot reports '${deployed}' — check, then push again"
  exit 0
fi
busy=$(side 90 busy 2>/dev/null)
[ "$busy" = idle ] || { note "waiting: robot ${busy:-busy-check failed}"; exit 0; }

log "deploy $stamp (robot had: ${deployed:-nothing})"
bk="/home/agilex/H-infinity-deploy-backups/$(date -u +%Y%m%dT%H%M%SZ)"
items=$(rsync -az --itemize-changes --backup --backup-dir="$bk" \
          -e "ssh $SSH_OPTS" --exclude-from="$STAGE/tools/sync/code_excludes.txt" \
          "$STAGE/" "$ROBOT:$ROBOT_ROOT/" 2>>"$LOG") || { log "rsync to robot FAILED"; exit 1; }
changed=$(printf '%s\n' "$items" | grep -E '^<f' | sed -E 's/^[^ ]+ //')
n=$(printf '%s' "$changed" | grep -c .)
log "copied $n file(s); overwritten robot files saved in $bk"

# Docs, photos, the browser page (opened on the laptop), this deployer and the
# test suites never run on the robot: they need no build/restart.
printf '%s\n' "$changed" \
  | grep -vE '^(DOC/|Photos/|tools/sync/|tools/qc/)|/tests?/|\.(md|zip|png|jpe?g|pdf|html)$' \
  | grep -q . && touch "$PENDING"
printf '%s\n' "$changed" | grep -q '^scalecar-vfg-h-infinite/vfg_pathfollowing/' \
  && touch "$PENDING_VENDOR"

if [ -e "$PENDING" ]; then
  vendor=0; [ -e "$PENDING_VENDOR" ] && vendor=1
  if ! out=$(side 900 build "$vendor" 2>&1); then
    log "BUILD FAILED for $stamp:"; printf '%s\n' "$out" | tail -15 >> "$LOG"
    echo "$stamp" > "$RELAY/failed.stamp"; note "holding: deploy failed for $stamp — push a fix"
    exit 1
  fi
  log "built (vendor reinstall: $vendor): $(printf '%s\n' "$out" | grep -E 'Summary|Finished' | tail -1)"
  rm -f "$PENDING_VENDOR"
  # A run may have started during the build: check again right before the restart.
  busy=$(side 90 busy 2>/dev/null)
  [ "$busy" = idle ] || { note "built $stamp; restart deferred: robot ${busy:-busy-check failed}"; exit 0; }
  if ! out=$(side 90 restart 2>&1); then log "RESTART FAILED: $out"; exit 1; fi
  log "$out"
  rm -f "$PENDING"
fi
echo "$stamp" > "$RELAY/delivered.stamp"
side 30 set-stamp "$stamp" || { log "could not record stamp on robot"; exit 1; }
# Read it back: "in sync" is only claimed for what the robot actually reports.
back=$(side 30 stamp 2>/dev/null)
if [ "$back" != "$stamp" ]; then
  log "STAMP MISMATCH after deploy: wrote '$stamp', robot reports '$back'"
  echo "$stamp" > "$RELAY/failed.stamp"; note "holding: deploy failed for $stamp — push a fix"
  exit 1
fi
rm -f "$RELAY/failed.stamp"
log "deployed $stamp"
note "in sync: $stamp"
