from setuptools import setup

package_name = 'robotWalk'

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
    maintainer='jordan',
    maintainer_email='reed5204@vandals.uidaho.edu',
    description='Taking the robot for a walk',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotWalk = robotWalk.main:main'
        ],
    },
)
