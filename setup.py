from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agrobot_pkg'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_sim.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/pathfinding_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/agrobot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/agrobot_model.obj']))
data_files.append(('share/' + package_name + '/resource', ['resource/carrot_model.obj']))
data_files.append(('share/' + package_name + '/protos', ['protos/PlanterBox.proto']))
data_files.append((os.path.join('share', package_name), glob('launch/*.launch_sim.py')))
data_files.append(('share/' + package_name + '/resource', ['resource/soil_texture.jpg']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools',
                      'rclpy',
                      'geometry_msgs',
                      'webots_ros2_driver'
                      'odrive_can',
                      'agrobot_msgs'],
    zip_safe=True,
    maintainer='tjacobs',
    maintainer_email='tjacobs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_controller_node = agrobot_pkg.simulation_controller_node:main',
            'movement_controller_node = agrobot_pkg.movement_controller_node:main',
            'vision_processing_node = agrobot_pkg.vision_processing_node:detect_object',
            'subscribe_test = agrobot_pkg.subscribe_test:subscribe_to_crop_info',
        ],
    },
)
