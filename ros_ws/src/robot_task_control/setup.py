from glob import glob
from setuptools import find_packages, setup

package_name = "robot_task_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/tools", glob("tools/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Execute local chassis and Elite robot YAML workflows.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hk_image_transfer = robot_task_control.hk_image_transfer:main",
            "velocity_mux = robot_task_control.velocity_mux:main",
            "task_controller = robot_task_control.workflow_controller:main",
        ],
    },
)
