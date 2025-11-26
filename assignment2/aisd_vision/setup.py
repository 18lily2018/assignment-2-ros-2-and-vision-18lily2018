from setuptools import setup

package_name = 'aisd_vision'

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
    description='Image publisher and hand detector nodes for CST8504 Assignment 2.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = aisd_vision.image_publisher:main',
            'hands = aisd_vision.hands:main',
        ],
    },
)
