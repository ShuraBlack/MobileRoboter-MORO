from setuptools import setup

package_name = 'my_jk_rollin_kinematics'

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
    maintainer='ro161klo',
    maintainer_email='ro161klo@htwg-konstanz.de',
    description='Inverse Kinematic calculation for jk_rollin',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jk_rollin_arm_ik = my_jk_rollin_kinematics.jk_rollin_arm_ik:main',
            'jk_rollin_draw_circle = my_jk_rollin_kinematics.jk_rollin_draw_circle:main',
        ],
    },
)
