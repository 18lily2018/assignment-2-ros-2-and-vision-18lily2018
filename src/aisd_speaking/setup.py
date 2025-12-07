from setuptools import setup

package_name = 'aisd_speaking'

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
    maintainer='aisd',
    maintainer_email='aisd@example.com',
    description='Speaking node for Assignment 3',
    license='none',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speak = aisd_speaking.speak:main',
        ],
    },
)
