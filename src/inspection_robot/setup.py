from glob import glob

from setuptools import find_packages, setup


package_name = "inspection_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iori",
    maintainer_email="99437733+iori-shino@users.noreply.github.com",
    description="A ROS 2 Python inspection task simulation using turtlesim.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "patrol_controller = inspection_robot.patrol_controller:main",
            "inspection_detector = inspection_robot.inspection_detector:main",
            "inspection_logger = inspection_robot.inspection_logger:main",
        ],
    },
)
