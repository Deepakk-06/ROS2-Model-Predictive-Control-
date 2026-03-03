from setuptools import setup
import os
from glob import glob

package_name = "mpc_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*")),
        (os.path.join("share", package_name, "waypoints"),
         glob("waypoints/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="MPC trajectory tracker with obstacle avoidance",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mpc_tracker     = mpc_nav.mpc_tracker:main",
            "path_smoother   = mpc_nav.path_smoother:main",
            "obstacle_spawner = mpc_nav.obstacle_spawner:main",
        ],
    },
)
