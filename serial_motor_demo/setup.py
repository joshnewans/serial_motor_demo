from setuptools import setup
import os
from glob import glob

package_name = 'serial_motor_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['utils.transformations'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='josh.newans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gui = serial_motor_demo.gui:main',
        'driver = serial_motor_demo.driver:main',
        'motor_command_node = serial_motor_demo.motor_command_node:main',
        'wheels_odometry = serial_motor_demo.wheels_odometry:main',
    ],
},

)
