"""
Pytest configuration and shared fixtures for AMR Omni E2E Test Suite.
"""
import os
import sys
from pathlib import Path
import pytest

# Ensure repository root and tests/ are in python sys.path
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(WORKSPACE_ROOT))
sys.path.insert(0, str(WORKSPACE_ROOT / "src" / "omni_control"))
sys.path.insert(0, str(WORKSPACE_ROOT / "web" / "backend"))


@pytest.fixture(scope="session")
def workspace_root() -> Path:
    return WORKSPACE_ROOT


@pytest.fixture
def temp_calib_dir(tmp_path: Path) -> Path:
    config_dir = tmp_path / "config"
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir
