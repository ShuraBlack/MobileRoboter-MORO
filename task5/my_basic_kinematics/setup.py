from setuptools import setup

package_name = 'my_basic_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronny Klotz',
    maintainer_email='ro161klo@htwg-konstanz.de',
    description='Verarbeitung von Sensorinformationen',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "visual_servoing = my_basic_kinematics.visual_servoing:main",
            "probabilistic_diffdrive = my_basic_kinematics.probabilistic_diffdrive:main",
            "plot_odom = my_basic_kinematics.plot_odom:main"
        ],
    },
)
