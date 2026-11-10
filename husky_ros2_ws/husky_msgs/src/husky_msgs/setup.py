from setuptools import find_packages, setup

package_name = 'husky_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='hotgluemaster3@gmail.com',
    description='Husky odometry and client telemetry publishers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'husky_telemetry_node = husky_msgs.husky_telemetry_node:main',
        ],
    },
)
