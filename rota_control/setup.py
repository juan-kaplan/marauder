from setuptools import find_packages, setup

package_name = 'rota_control'

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
    maintainer='seb',
    maintainer_email='sebabedin@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'diff_control_node = rota_control.diff_control_node:main',
            'teach_node = rota_control.teach_node:main',
            'repeat_node = rota_control.repeat_node:main',
        ],
    },
)

