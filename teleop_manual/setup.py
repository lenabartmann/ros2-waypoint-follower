from setuptools import setup


package_name = 'teleop_manual'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lena Bartmann',
    maintainer_email='lena.bartmann@iu-study.org',
    description='A ROS2 node allowing manual teleoperation of a mobile robot via velocity commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_manual = teleop_manual.teleop_manual:main',
        ],
    },
)
