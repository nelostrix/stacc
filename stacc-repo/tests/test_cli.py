"""Basic tests for stacc CLI."""
import subprocess
import sys
import json
import os
import shutil
import tempfile
import pytest


def run_stacc(*args):
    """Run stacc command and return result."""
    result = subprocess.run(
        [sys.executable, "-m", "stacc.cli"] + list(args),
        capture_output=True,
        text=True,
    )
    return result


class TestStaccVersion:
    def test_version_flag(self):
        result = run_stacc("--version")
        assert "0.1.0" in result.stdout

    def test_help_flag(self):
        result = run_stacc("--help")
        assert "stacc" in result.stdout
        assert "init" in result.stdout


class TestStaccInit:
    def setup_method(self):
        self.tmpdir = tempfile.mkdtemp()
        self.original_dir = os.getcwd()
        os.chdir(self.tmpdir)

    def teardown_method(self):
        os.chdir(self.original_dir)
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_init_blank(self):
        result = run_stacc("init", "test_robot", "--template", "blank")
        assert result.returncode == 0
        assert os.path.exists("test_robot/stacc.json")

    def test_init_with_template(self):
        result = run_stacc("init", "test_arm", "--template", "arm_6dof")
        assert result.returncode == 0
        assert os.path.exists("test_arm/src/robot.py")
        
        with open("test_arm/stacc.json") as f:
            manifest = json.load(f)
        assert manifest["name"] == "test_arm"
        assert manifest["robot"]["type"] == "manipulator"

    def test_init_all_templates(self):
        templates = ["blank", "line_follower", "arm_6dof", "quadruped", "rover", "drone"]
        for tpl in templates:
            result = run_stacc("init", f"test_{tpl}", "--template", tpl)
            assert result.returncode == 0, f"Failed for template: {tpl}"

    def test_init_duplicate_fails(self):
        run_stacc("init", "test_dup")
        result = run_stacc("init", "test_dup")
        assert result.returncode != 0


class TestStaccSearch:
    def test_search_navigation(self):
        result = run_stacc("search", "navigation")
        assert "slam-toolkit" in result.stdout
        assert "path-planning" in result.stdout

    def test_search_no_results(self):
        result = run_stacc("search", "nonexistent_xyz")
        assert "No packages found" in result.stdout


class TestStaccDoctor:
    def test_doctor_runs(self):
        result = run_stacc("doctor")
        assert result.returncode == 0
        assert "Python" in result.stdout


class TestStaccInstall:
    def setup_method(self):
        self.tmpdir = tempfile.mkdtemp()
        self.original_dir = os.getcwd()
        os.chdir(self.tmpdir)
        run_stacc("init", "test_project")
        os.chdir("test_project")

    def teardown_method(self):
        os.chdir(self.original_dir)
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_install_package(self):
        result = run_stacc("install", "path-planning")
        assert result.returncode == 0
        
        with open("stacc.json") as f:
            manifest = json.load(f)
        assert "path-planning" in manifest["dependencies"]

    def test_install_with_deps(self):
        result = run_stacc("install", "slam-toolkit")
        assert result.returncode == 0
        
        with open("stacc.json") as f:
            manifest = json.load(f)
        assert "slam-toolkit" in manifest["dependencies"]
        assert "eigen-bindings" in manifest["dependencies"]
        assert "perception-core" in manifest["dependencies"]

    def test_install_nonexistent(self):
        result = run_stacc("install", "fake-package-xyz")
        assert result.returncode != 0
