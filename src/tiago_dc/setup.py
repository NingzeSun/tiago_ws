from glob import glob

from setuptools import setup

package_name = "tiago_dc"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/web", glob(package_name + "/web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="n7sun",
    maintainer_email="ningze1005@gmail.com",
    description="Meta Quest teleoperation and RoboTwin-format data collection for TIAGo Pro",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_node = tiago_dc.teleop_node:main",
            "recorder_node = tiago_dc.recorder_node:main",
            "quest_webxr_node = tiago_dc.quest_webxr_node:main",
            "scene_node = tiago_dc.scene_node:main",
        ],
    },
)
