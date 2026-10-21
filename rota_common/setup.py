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
    description='Shared helpers for the rota_* packages (transforms, cmd_vel publishing).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
