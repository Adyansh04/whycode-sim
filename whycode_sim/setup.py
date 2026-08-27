from glob import glob
from os.path import isfile

from setuptools import find_packages, setup

package_name = "whycode_sim"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # .rviz as well as .yaml: RViz is handed this path and falls back to its
        # defaults without an error if the file is not there.
        (
            f"share/{package_name}/config",
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        # Files only: glob("scripts/*") also matches subdirectories, which setuptools
        # cannot copy as data files.
        (f"share/{package_name}/scripts", [p for p in glob("scripts/*") if isfile(p)]),
        (
            f"share/{package_name}/scripts/decode_marker_ids",
            [p for p in glob("scripts/decode_marker_ids/*") if isfile(p)],
        ),
        (f"share/{package_name}/textures/whycode", glob("textures/whycode/*.png")),
        (f"share/{package_name}/textures/apriltag", glob("textures/apriltag/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adyansh Gupta",
    maintainer_email="gupta.adyansh@gmail.com",
    description="Isaac Sim warehouse benchmark scene for WhyCode and AprilTag detection.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "ground_truth_node = whycode_sim.ground_truth_node:main",
            "waypoint_follower_node = whycode_sim.waypoint_follower_node:main",
            "ground_truth_viz_node = whycode_sim.ground_truth_viz_node:main",
        ],
    },
)
