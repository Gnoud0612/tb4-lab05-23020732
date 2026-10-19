from glob import glob
from setuptools import find_packages, setup

setup(
    name='tb4_lab',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tb4_lab']),
        ('share/tb4_lab', ['package.xml']),
        ('share/tb4_lab/launch', glob('launch/*.launch.py')),
        ('share/tb4_lab/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyễn Tùng Dương',
    maintainer_email='student@example.com',
    description='Practical TurtleBot 4 simulation lab on ROS 2 Humble',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'check_topics = tb4_lab.check_topics:main',
        'goal_monitor = tb4_lab.goal_monitor:main',
        'undock = tb4_lab.undock:main',
        'check_localization = tb4_lab.check_localization:main',
    ]},
)
