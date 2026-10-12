from glob import glob
from setuptools import find_packages, setup

package_name = "eli_to_chassis_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # launch 通过脚本文件启动，不依赖 editable 包路径或入口元数据。
        ("share/" + package_name + "/scripts", [
            "eli_to_chassis_control/workflow_controller.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Coordinate a chassis task with delayed robot pose execution.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "workflow_controller = eli_to_chassis_control.workflow_controller:main",
        ],
    },
)
