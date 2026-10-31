from setuptools import find_packages, setup

package_name = "pickorange_chassis"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "msgpack"],
    zip_safe=True,
    maintainer="BohnChen",
    maintainer_email="bohn@example.com",
    description="底盘桥接：/cmd_vel <-> Arduino RouterBridge RPC",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_node = pickorange_chassis.chassis_node:main",
        ],
    },
)
