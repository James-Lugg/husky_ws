from setuptools import find_packages, setup

package_name = "jy901b_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="James Lugg",
    maintainer_email="james@example.com",
    description="ROS 2 serial driver for the WitMotion JY901B IMU",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "jy901b_node = jy901b_driver.jy901b_node:main",
        ],
    },
)
