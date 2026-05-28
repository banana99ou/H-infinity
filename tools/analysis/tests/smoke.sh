#!/usr/bin/env bash
# Full smoke test of the bag -> dataset analysis pipeline (laptop, no ROS).
# Regenerates synthetic fixtures and runs every stage end-to-end, asserting the
# expected artifacts appear. Exits 0 only if every check passes.
#
#   bash tools/analysis/tests/smoke.sh
#
# Requires: python3 + rosbags, pyarrow, scipy, numpy, matplotlib, pyyaml.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ANALYSIS="$(dirname "$HERE")"
REPO="$(cd "$ANALYSIS/../.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

PASS=0; FAIL=0
ok()   { printf '  [PASS] %s\n' "$1"; PASS=$((PASS+1)); }
bad()  { printf '  [FAIL] %s\n' "$1"; FAIL=$((FAIL+1)); }
chk()  { if eval "$2"; then ok "$1"; else bad "$1 ($2)"; fi; }

cd "$REPO"

echo "== 1. fixtures =="
python3 tools/analysis/tests/make_fixture.py "$WORK/single" --clean >/dev/null 2>&1
chk "single good fixture bag created"   "[ -f '$WORK/single/metadata.yaml' ]"
chk "single sidecar created"            "[ -f '$WORK/single.sidecar.json' ]"
python3 tools/analysis/tests/make_fixture.py --batch "$WORK/bagroot" >/dev/null 2>&1
chk "batch produced 16 leg bags"        "[ \$(find '$WORK/bagroot' -name metadata.yaml | wc -l) -eq 16 ]"

echo "== 2. run_eval (one leg) =="
python3 tools/analysis/run_eval.py "$WORK/single" >/dev/null 2>&1
chk "metrics JSON written"              "[ -f '$WORK/single.metrics.json' ]"
chk "rtk_truth rms_e_d ~ 0 (round-trip)" \
  "python3 -c \"import json;d=json.load(open('$WORK/single.metrics.json'));import sys;sys.exit(0 if d['metrics']['rtk_truth']['rms_e_d']<1e-6 else 1)\""
chk "cell_params passed through" \
  "python3 -c \"import json;d=json.load(open('$WORK/single.metrics.json'));import sys;sys.exit(0 if d.get('cell_params') else 1)\""
chk "odom-belief sourced from /wheel/odom_zeroed" \
  "python3 -c \"import json;d=json.load(open('$WORK/single.metrics.json'));import sys;sys.exit(0 if d.get('odom_belief_source')=='/wheel/odom_zeroed' else 1)\""

echo "== 3. manifest + qc (RTK mode) =="
python3 tools/analysis/manifest.py "$WORK/bagroot" --out-dir "$WORK/d" >/dev/null 2>&1
chk "manifest.csv written"              "[ -f '$WORK/d/manifest.csv' ]"
python3 tools/analysis/qc.py "$WORK/bagroot" --out-dir "$WORK/d" >/dev/null 2>&1
chk "qc marks all 16 batch legs usable" \
  "[ \$(python3 -c \"import json;print(len(json.load(open('$WORK/d/usable_legs.json'))))\") -eq 16 ]"

echo "== 4. qc indoor mode (--no-rtk-gate on a bad-RTK leg) =="
python3 tools/analysis/tests/make_fixture.py "$WORK/badrtk" --rtk-bad >/dev/null 2>&1
python3 tools/analysis/qc.py "$WORK" --no-rtk-gate --out-dir "$WORK/d_indoor" >/dev/null 2>&1
chk "bad-RTK leg usable under --no-rtk-gate" \
  "python3 -c \"import json,glob;u=json.load(open('$WORK/d_indoor/usable_legs.json'));import sys;sys.exit(0 if any('badrtk' in p for p in u) else 1)\""

echo "== 5. build_dataset (RTK mode, full driver) =="
python3 tools/analysis/build_dataset.py "$WORK/bagroot" --out "$WORK/ds" >/dev/null 2>&1
chk "raw/ layer pointer"                "[ -f '$WORK/ds/raw/SOURCE.txt' ]"
chk "derived headline figure"           "[ -f '$WORK/ds/derived/headline_v1_step.png' ]"
chk "derived cell_summary.csv"          "[ -f '$WORK/ds/derived/cell_summary.csv' ]"
chk "derived stats.json"                "[ -f '$WORK/ds/derived/stats.json' ]"
chk "extracted per_sample tables (32)"  "[ \$(ls '$WORK/ds/extracted/per_sample' | wc -l) -eq 32 ]"
chk "extracted GNSS product (32)"       "[ \$(ls '$WORK/ds/extracted/gnss' | wc -l) -eq 32 ]"
chk "data dictionary"                   "[ -f '$WORK/ds/extracted/data_dictionary.md' ]"
chk "Wilcoxon ran in stats.json" \
  "python3 -c \"import json;s=json.load(open('$WORK/ds/derived/stats.json'));import sys;sys.exit(0 if s['slices']['v1_step']['wilcoxon_lpv_vs_pid']['ok'] else 1)\""

echo "== 6. build_dataset (indoor / odom-belief) =="
python3 tools/analysis/build_dataset.py "$WORK/bagroot" --out "$WORK/ds_indoor" --no-rtk-gate >/dev/null 2>&1
chk "indoor headline figure"            "[ -f '$WORK/ds_indoor/derived/headline_v1_step.png' ]"

echo
echo "smoke: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
