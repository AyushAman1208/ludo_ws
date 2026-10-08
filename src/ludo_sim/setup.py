import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ludo_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
        (os.path.join("share", package_name, "models", "robots"), glob("models/robots/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="MuJoCo simulation of a Ludo board played by four robot arms with cameras.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mujoco_node = ludo_sim.mujoco_node:main",
            "state_publisher = ludo_sim.state_publisher:main",
        ],
    },
)
