from setuptools import find_packages, setup

package_name = 'kinematics_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edu',
    maintainer_email='eduisant@espol.edu.ec',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematics_node = kinematics_controller.kinematics_node:main',
            'teleop_node = kinematics_controller.custom_teleop:main',
            'vxy_logger = kinematics_controller.vxy_logger:main',
            'manual_logger = kinematics_controller.manual_trajectory_logger:main'
        ],
    },
)
