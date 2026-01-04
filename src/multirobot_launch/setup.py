import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multirobot_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/multirobot_launch']),
    ('share/multirobot_launch', ['package.xml']),
    (os.path.join('share', 'multirobot_launch', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sathvik',
    maintainer_email='awskrishaka@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        
        ],
    },
)
