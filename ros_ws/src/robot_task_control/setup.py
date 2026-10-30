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
        *[("share/" + package_name + "/" + folder, glob(folder + "/*.launch.py"))
          for folder in ("launch", "launch/internal", "launch/tests")],
        *[("share/" + package_name + "/" + folder,
           glob(folder + "/*.yaml") + glob(folder + "/*.json"))
          for folder in ("config", "config/capture", "config/tests")],
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Execute local chassis and Elite robot YAML workflows.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hk_image_transfer = industrial_pc_hk_transfer:main",
            "velocity_mux = robot_task_control.velocity_mux:main",
            "task_controller = robot_task_control.workflow_controller:main",
            "pixel_point_simulation = robot_task_control.testing.pixel_point_simulation:main",
            "camera_detection_task = robot_task_control.camera_detection_task:main",
            "virtual_detection_task = robot_task_control.testing.virtual_detection_task:main",
        ],
    },
)
