"""Path setup so ``pytest tests/qc`` works without the run_qc.py wrapper.

The tests import repo modules three ways:
  * ``import Data_Logger`` / ``from tools.analysis import ...`` -> need REPO_ROOT
  * ``from vfg_pathfollowing... import ...`` (via run_eval) -> need the vendored
    ``scalecar-vfg-h-infinite/`` on sys.path
  * ``import path_overlay`` (lazy, inside run_eval) -> need ``tools/path_gen/``

``tools/qc/run_qc.py`` injects these via PYTHONPATH for the tiered runner; this
conftest makes a bare ``pytest tests/qc`` (or ``python3 -m pytest``) discover and
import the same way, so test collection is invocation-independent.
"""

from __future__ import annotations

import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

for _p in (
    _REPO_ROOT,
    os.path.join(_REPO_ROOT, "scalecar-vfg-h-infinite"),
    os.path.join(_REPO_ROOT, "tools", "path_gen"),
):
    if _p not in sys.path:
        sys.path.insert(0, _p)
