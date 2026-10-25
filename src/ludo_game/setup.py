import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ludo_game"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Publishes dice rolls for player nodes in the Ludo environment.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dice_publisher = ludo_game.dice_publisher:main",
        ],
    },
)
