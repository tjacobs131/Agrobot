import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('agrobot_pkg')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'agrobot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'pathfinding_world.wbt')
    )

    robot_driver = WebotsController(
        robot_name='Agrobot',
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    movement_controller = Node(
        package="agrobot_pkg",
        name="movement_controller_node",
        executable="movement_controller_node"
    )

    return LaunchDescription([
        webots,
        robot_driver,
        movement_controller,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])