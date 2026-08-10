from glob import glob
import os

from setuptools import find_packages, setup


package_name = "stereo_nav_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stereo_nav_sim",
    maintainer_email="maintainer@example.com",
    description="Pure-stereo RTAB-Map and Nav2 bringup.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "moving_obstacle_controller = stereo_nav_bringup.moving_obstacle_controller:main",
        ],
    },
)
