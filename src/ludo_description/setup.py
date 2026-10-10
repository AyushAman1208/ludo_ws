from setuptools import find_packages, setup

package_name = "ludo_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Canonical robot and board geometry shared across the Ludo workspace.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generate_robot_models = ludo_description.model_codegen:main",
        ],
    },
)
