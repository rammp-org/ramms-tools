"""Packaging constraints that a robot install depends on (issue #13).

These are not style checks. `rammp-UI-bridge` pip-installs this package into
the system Python of a ROS 2 Humble robot, so the version constraints here
decide whether that install quietly breaks the host:

- Raising the NumPy floor above 1.21 makes pip upgrade the system NumPy, and
  Ubuntu 22.04's `python3-scipy` 1.8.0 is built against NumPy < 1.25, so every
  ROS node importing SciPy then dies on a binary incompatibility.
- Dropping the setuptools upper bound breaks the colcon build on the robot.

Both regressions look harmless in a diff and only show up on the robot, which
is what these tests are for.
"""
from __future__ import annotations

import os
import pathlib
import subprocess
import sys
import textwrap

import pytest

import ramms_tools

try:
    import tomllib  # Python >= 3.11
except ModuleNotFoundError:  # pragma: no cover - exercised on Python 3.10
    tomllib = pytest.importorskip(
        "tomli", reason="need tomllib (3.11+) or tomli to read pyproject.toml"
    )

PYPROJECT = pathlib.Path(__file__).resolve().parent.parent / "pyproject.toml"

# What Ubuntu 22.04 / ROS 2 Humble ships. The floor must not exceed this.
HUMBLE_NUMPY = (1, 21, 5)
# The highest setuptools that builds under colcon on the robot, per issue #13.
SETUPTOOLS_CEILING = (80,)


def _config() -> dict:
    with PYPROJECT.open("rb") as fh:
        return tomllib.load(fh)


def _parse_version(text: str) -> tuple[int, ...]:
    return tuple(int(part) for part in text.strip().split("."))


def _requirement(requirements: list[str], name: str) -> str:
    for req in requirements:
        if req.lower().startswith(name):
            return req
    raise AssertionError(f"{name!r} is not declared in {requirements!r}")


def test_numpy_floor_does_not_exceed_what_ros_humble_ships():
    numpy_req = _requirement(_config()["project"]["dependencies"], "numpy")
    floors = [
        _parse_version(clause.split(">=", 1)[1])
        for clause in numpy_req.split(",")
        if ">=" in clause
    ]
    assert floors, f"no lower bound found in {numpy_req!r}"
    assert max(floors) <= HUMBLE_NUMPY, (
        f"{numpy_req!r} would make pip upgrade the NumPy that ROS 2 Humble ships "
        f"({'.'.join(map(str, HUMBLE_NUMPY))}), which breaks the system SciPy"
    )


def test_setuptools_build_requirement_is_capped():
    setuptools_req = _requirement(_config()["build-system"]["requires"], "setuptools")
    ceilings = [
        _parse_version(clause.split("<", 1)[1])
        for clause in setuptools_req.split(",")
        if "<" in clause and "<=" not in clause
    ]
    assert ceilings, (
        f"{setuptools_req!r} has no upper bound; the colcon build on the robot "
        f"needs setuptools < {'.'.join(map(str, SETUPTOOLS_CEILING))}"
    )
    assert min(ceilings) <= SETUPTOOLS_CEILING


def test_version_is_single_sourced_from_the_package():
    """A release tag, the dist metadata and the module must not disagree."""
    config = _config()
    assert "version" in config["project"].get("dynamic", []), (
        "pyproject should take its version from ramms_tools.__version__, not "
        "repeat it -- two literals drift the moment one is bumped"
    )
    attr = config["tool"]["setuptools"]["dynamic"]["version"]["attr"]
    assert attr == "ramms_tools.__version__", attr


def test_runtime_dependencies_are_importable_without_the_extras():
    """What rammp-UI-bridge imports must work with the declared deps alone.

    Pillow and lz4 are optional, and ramms_tools.streaming.compression guards
    both with try/except ImportError. Asserting that here in-process would
    prove nothing: a development machine has them installed, so an unguarded
    import would succeed locally and only fail on the robot. Run it in a fresh
    interpreter with those two packages made unimportable instead.
    """
    blocked = textwrap.dedent(
        """
        import sys

        class Blocker:
            def find_spec(self, name, path=None, target=None):
                if name.split(".")[0] in {"PIL", "lz4"}:
                    raise ImportError("blocked for test: " + name)
                return None

        sys.meta_path.insert(0, Blocker())

        from ramms_tools.streaming import StreamSender, has_jpeg, has_lz4

        assert StreamSender is not None, "StreamSender did not import"
        assert has_jpeg() is False, "has_jpeg() should be False without Pillow"
        assert has_lz4() is False, "has_lz4() should be False without lz4"
        print("OK")
        """
    )

    # Make ramms_tools importable in the child whether the tests run against an
    # installed package or a source checkout.
    package_root = pathlib.Path(ramms_tools.__file__).resolve().parent.parent
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(package_root), env.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)

    result = subprocess.run(
        [sys.executable, "-c", blocked],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, (
        "importing ramms_tools.streaming without Pillow/lz4 failed -- an "
        f"optional dependency is imported unguarded:\n{result.stderr}"
    )
    assert "OK" in result.stdout, result.stdout
