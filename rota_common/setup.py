from setuptools import find_packages, setup

package_name = 'rota_common'

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
    maintainer='juan-kaplan',
    maintainer_email='jkaplan@udesa.edu.ar',
    description='Shared library for the rota_* packages: SE(2) poses, message conversions, scans, ICP, cmd_vel publishing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
