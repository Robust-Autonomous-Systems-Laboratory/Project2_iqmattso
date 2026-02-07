import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gimbal_lock_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quin',
    maintainer_email='iqmattso@mtu.edu',
    description='Use rqt_reconfigure and the robot state publisher to control rotations of a URDF with euler angles and quaternions',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'euler_quaternion_state_publisher = gimbal_lock_demo.euler_quaternion_state_publisher:main',
        ],
    },
)
