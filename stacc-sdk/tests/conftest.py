"""Script-style test files run their checks at import time and finish with
sys.exit(), which aborts a normal pytest collection. They are not imported;
test_script_suites.py runs each one as a subprocess instead.

The SDK is used from source (not pip-installed), so it is put on PYTHONPATH
for every subprocess a test starts (e.g. `python -m stacc.cli`)."""

import os
from pathlib import Path

_SDK = str(Path(__file__).resolve().parent.parent)
os.environ["PYTHONPATH"] = os.pathsep.join(filter(None, [_SDK, os.environ.get("PYTHONPATH")]))

SCRIPT_SUITES = [
    "test_ai.py",
    "test_all_sensors.py",
    "test_master.py",
    "test_master_recorder_behavior_tee_hal_world.py",
    "test_rss.py",
]

collect_ignore = list(SCRIPT_SUITES)
