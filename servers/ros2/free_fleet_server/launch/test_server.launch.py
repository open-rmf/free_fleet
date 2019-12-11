
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # .yaml file for configuring the parameters
    yaml = os.path.join(
        get_package_share_directory('free_fleet_server'), 
            'launch', 'test_server.yaml'
    )

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='free_fleet_server', node_executable='free_fleet_server', 
            output='screen', arguments=['--ros-args --params-file '+yaml]),
    ])
