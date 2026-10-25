"""Runs the script-style suites (see conftest.py) and requires a clean exit."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

from conftest import SCRIPT_SUITES

HERE = Path(__file__).resolve().parent
SDK = HERE.parent


@pytest.mark.parametrize("script", SCRIPT_SUITES)
def test_script_suite(script):
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(filter(None, [str(SDK), os.environ.get("PYTHONPATH")]))}
    proc = subprocess.run([sys.executable, str(HERE / script)], cwd=SDK, env=env, capture_output=True, text=True, timeout=900)
    assert proc.returncode == 0, f"{script} exited {proc.returncode}\n{proc.stdout[-4000:]}\n{proc.stderr[-4000:]}"
