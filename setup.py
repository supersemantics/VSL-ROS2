"""vsl-ros2 packaging, for both pip (PyPI wheel) and colcon (ament_python).

One file, because colcon's ament_python build runs the system setuptools,
which on Humble (59) cannot read a pyproject [project] table.
"""

from pathlib import Path

from setuptools import find_packages, setup

PACKAGE = "vsl_ros2"
HERE = Path(__file__).parent


def _data_files():
    files = [
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/config", sorted(str(p.relative_to(HERE)) for p in (HERE / "config").glob("*.yaml"))),
        (f"share/{PACKAGE}/launch", sorted(str(p.relative_to(HERE)) for p in (HERE / "launch").glob("*.launch.py"))),
    ]
    return [(dest, [s.replace("\\", "/") for s in srcs]) for dest, srcs in files if srcs]


setup(
    name="vsl-ros2",
    version="0.1.0",
    description=(
        "ROS 2 (rclpy) adapter for VSL (VERBA Specification Language) -- reads ROS 2's own verdicts "
        "(Nav2 collision monitor, diagnostics, lifecycle, e-stop) as vsl-core Invariants and adds what "
        "ROS 2 lacks: gated tasks, a decision ledger, human-authorised Terminal States."
    ),
    long_description=(HERE / "README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    author="Super Semantics",
    license="MIT",
    url="https://github.com/supersemantics/VSL-ROS2",
    project_urls={
        "Repository": "https://github.com/supersemantics/VSL-ROS2",
        "Changelog": "https://github.com/supersemantics/VSL-ROS2/blob/HEAD/CHANGELOG.md",
        "Issues": "https://github.com/supersemantics/VSL-ROS2/issues",
    },
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    data_files=_data_files(),
    python_requires=">=3.10",
    install_requires=[
        # vsl-core, published on PyPI as super-semantics-vsl (imported as vsl_core);
        # vsl_ros2 checks the installed version on import (needs >= 0.3.0).
        # rclpy is deliberately not listed: it is not on PyPI and comes with ROS 2.
        "super-semantics-vsl>=0.3,<0.4",
    ],
    # "test" is the extra colcon looks for to run pytest; "dev" is kept for pip users.
    extras_require={"test": ["pytest"], "dev": ["pytest>=7"]},
    entry_points={
        "console_scripts": [
            "vsl_guard = vsl_ros2.guard_node:main",
            "vsl_guard_release = vsl_ros2.release_cli:main",
            "vsl_guard_report = vsl_ros2.report:main",
        ],
    },
    keywords=["governance", "ai-safety", "guardrails", "ros2", "rclpy", "robotics", "vsl", "verba"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Development Status :: 3 - Alpha",
    ],
    zip_safe=True,
)
