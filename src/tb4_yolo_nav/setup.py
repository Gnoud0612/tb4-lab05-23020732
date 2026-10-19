from setuptools import setup, find_packages
from glob import glob

setup(
    name="tb4_yolo_nav",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tb4_yolo_nav"]),
        ("share/tb4_yolo_nav", ["package.xml"]),
        ("share/tb4_yolo_nav/launch", glob("launch/*.launch.py")),
        ("share/tb4_yolo_nav/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nguyễn Tùng Dương",
    maintainer_email="student@example.com",
    description="TurtleBot 4 sequential autonomous navigation and camera YOLO",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_mission = tb4_yolo_nav.waypoint_mission:main",
            "yolo_detector = tb4_yolo_nav.yolo_detector:main",
            "record_goals = tb4_yolo_nav.record_goals:main",
        ]
    },
)
