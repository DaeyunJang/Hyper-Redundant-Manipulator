from setuptools import find_packages, setup
import glob
import os

package_name = 'serial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch*.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daeyun',
    maintainer_email='bigyun9375@gmail.com',
    description='serial-read-package-for-forcesensor-and-loadcell-indicater',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_read = serial_pkg.serial_read:main',
            'serial_read_demo = serial_pkg.serial_read_demo:main'
        ],
    },
)
