from setuptools import setup

package_name = 'aisd_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='18lily2018',
    maintainer_email='your_email@algonquinlive.com',
    description='Move node that converts Hand messages into Twist commands for cmd_vel.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = aisd_motion.move:main',
        ],
    },
)
