import os
from glob import glob

from setuptools import setup

package_name = "lift_sim_skeleton"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Caleb Garcia",
    maintainer_email="melchorcaleb95@gmail.com",
    description="Skeleton OpenRMF lift node: a simulated lift for adapter testing.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lift_node = lift_sim_skeleton.lift_node:main",
            "lift_client_demo = lift_sim_skeleton.client_demo:main",
        ],
    },
)
