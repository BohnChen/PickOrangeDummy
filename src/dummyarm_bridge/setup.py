from setuptools import setup
import os
from glob import glob

package_name = 'dummyarm_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BohnChen',
    maintainer_email='59443600+BohnChen@users.noreply.github.com',
    description='Dummy 真机桥接节点 (阶段 3a)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dummyarm_system = dummyarm_bridge.bridge_node:main',
        ],
    },
)
