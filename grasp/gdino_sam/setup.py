from setuptools import setup
import os
from glob import glob

package_name = "gdino_sam"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all scripts in the scripts directory
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=[
        "setuptools",
        "readchar",  # Added readchar dependency
        # Removed 'keyboard' dependency
    ],
    zip_safe=True,
    maintainer="Zeng Shaoyu",
    maintainer_email="zshaoyu@connect.hku.hk",
    description="Detect and segment objects using GroundingDINO and SAM",
    license="Your License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "realsense_ros_test = gdino_sam.realsense_ros_test:main",
        ],
    },
)
