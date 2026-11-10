from setuptools import find_packages, setup

package_name = 'husky_joycon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='hotgluemaster3@gmail.com',
    description='Independent Husky joystick and cmd_vel controllers',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'husky_joycon_node = husky_joycon.husky_joycon_node:main',
            'husky_cmd_vel_node = husky_joycon.husky_cmd_vel_node:main',
        ],
    },
)
