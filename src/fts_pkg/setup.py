from setuptools import find_packages, setup
import glob
import os

package_name = 'fts_pkg'

setup(
    name=package_name,
    version='0.1.0',
    # packages=find_packages(exclude=['test']),
    packages=['fts_pkg', 'fts_pkg/fts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daeyun',
    maintainer_email='bigyun9375@gmail.com',
    description='Force Torque Sensor package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fts_node = fts_pkg.fts_read:main',
        ],
    },
)
