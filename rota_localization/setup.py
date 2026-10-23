from setuptools import find_packages, setup
from glob import glob

package_name = 'rota_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zoe',
    maintainer_email='zvelazquezzorzi@udesa.edu.ar',
    description='Layer 1 of Teach and Repeat: local localization from encoders, LiDAR or both.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_node = rota_localization.localization_node:main',
        ],
    },
)
