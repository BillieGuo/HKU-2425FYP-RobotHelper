from setuptools import setup
import os
from glob import glob

package_name = "anygrasp"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name, "license"), glob("license/*")),
        (os.path.join("lib", package_name), glob("lib/*.so")),
        (os.path.join("share", package_name, "log"), glob("log/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zeng Shaoyu",
    maintainer_email="zshaoyu@connect.hku.hk",
    description="AnyGrasp ROS2 package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "anygrasp_node = anygrasp.anygrasp_node:main",
            "grasp_responder = anygrasp.grasp_responder:main",
        ],
    },
)
