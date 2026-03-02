from setuptools import setup


package_name = 'lane_follower'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lena Bartmann',
    maintainer_email='lena.bartmann@iu-study.org',
    description='A simple ROS2 node for autonomous waypoint navigation of a mobile robot using odometry feedback',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = lane_follower.waypoint_follower:main',
        ],
    },
)
