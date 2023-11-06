from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_pkg'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_sim.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/agrobot.urdf']))
data_files.append((os.path.join('share', package_name), glob('launch/*.launch_sim.py')))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools',
                      'rclpy',
                      'geometry_msgs',
                      'webots_ros2_driver'],
    zip_safe=True,
    maintainer='tjacobs',
    maintainer_email='tjacobs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_driver_node = agrobot_pkg.movement_driver_node:main',
        ],
    },
)
