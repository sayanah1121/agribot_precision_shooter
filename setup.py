from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agribot_precision_sprayer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Precision agriculture robot using Pi 5 and Arduino',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = agribot_precision_sprayer.weed_detector:main',
            'manager = agribot_precision_sprayer.weed_manager:main',
            'bridge = agribot_precision_sprayer.serial_bridge:main',
        ],
    },
)
