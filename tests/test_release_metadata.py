"""Release metadata and public documentation consistency checks."""

import re
import tomllib
from pathlib import Path

import onero_sdk


ROOT = Path(__file__).resolve().parents[1]


def test_release_version_is_0_3_1_everywhere():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())

    assert project["project"]["version"] == "0.3.1"
    assert onero_sdk.__version__ == "0.3.1"


def test_public_repository_url_points_to_current_github_repository():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())

    assert project["project"]["urls"]["Repository"] == (
        "https://github.com/OpenWonderLabs/onero-sdk"
    )


def test_public_gripper_examples_use_onero_gripper():
    paths = [
        ROOT / "README.md",
        ROOT / "README_en.md",
        ROOT / "examples" / "gripper_pick_place.py",
    ]

    for path in paths:
        text = path.read_text()
        assert "OneroGripper" in text, path
        assert "OmniPickerGripper" not in text, path


def test_readmes_display_release_version():
    for name in ("README.md", "README_en.md"):
        text = (ROOT / name).read_text()
        assert re.search(r"\b0\.3\.1\b", text), name
