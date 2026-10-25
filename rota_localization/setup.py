from setuptools import find_packages, setup

package_name = 'rota_localization'

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
    maintainer='zoe',
    maintainer_email='zvelazquezzorzi@udesa.edu.ar',
    description='Local localization from encoders, LiDAR or both (layer 1 of Teach and Repeat), plus wheel and LiDAR odometry.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_node = rota_localization.localization_node:main',
            'wheel_odometry_node = rota_localization.wheel_odometry_node:main',
            'lidar_odometry_node = rota_localization.lidar_odometry_node:main',
        ],
    },
)
