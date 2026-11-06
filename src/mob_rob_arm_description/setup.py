import os
from glob import glob
from setuptools import setup

package_name = 'mob_rob_arm_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edu',
    maintainer_email='teduisant@espol.edu.ec',
    description='Descripcion del robot mob_rob_arm (base + UR10 + LiDAR)',
    license='BSD',
    entry_points={'console_scripts': []},
)