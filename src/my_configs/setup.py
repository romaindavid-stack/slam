from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_configs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # This ensures everything in launch/*.py goes into the /share/package/launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Same for config
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        # # This grabs everything in your launch folder
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # # This grabs everything in your config folder
        # (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romain',
    maintainer_email='romain.david@talpainspection.ch',
    description='Custom SLAM configurations and launch wrappers',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)