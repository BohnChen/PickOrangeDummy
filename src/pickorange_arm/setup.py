from setuptools import find_packages, setup

package_name = 'pickorange_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arduino',
    maintainer_email='arduino@todo.todo',
    description='ROS 2 Action Server for PickOrange DummyArm: MoveToPose and CutStem',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_action_server = pickorange_arm.arm_action_server:main',
        ],
    },
)
