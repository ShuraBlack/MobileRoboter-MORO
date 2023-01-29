from setuptools import setup
import os
from glob import glob

package_name = 'my_diffdrive_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, 'launch'), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name, 'rviz'), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, 'config'), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ro161klo',
    maintainer_email='ro161klo@htwg-konstanz.de',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "straight_drive = my_diffdrive_kinematics.straight_drive:main",
            "curve_drive = my_diffdrive_kinematics.curve_drive:main",
            "approach_goal = my_diffdrive_kinematics.approach_goal:main",
            "generate_path = my_diffdrive_kinematics.generate_path:main",
            "line_follower = my_diffdrive_kinematics.line_follower:main"
        ],
    },
)
