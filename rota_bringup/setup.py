from glob import glob
from setuptools import setup

package_name = 'rota_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan-kaplan',
    maintainer_email='jkaplan@udesa.edu.ar',
    description='Launch files, RViz configs and robot profiles for the rota_* nodes, and the Gazebo simulation.',
    license='Apache-2.0',
)
