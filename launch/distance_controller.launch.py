import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # rviz_config = os.path.join(get_package_share_directory('distance_controller'),'rviz','rosbotxl_sim.rviz')

    distance_controller_node = Node(
        package='distance_controller',
        executable='distance_controller',
        name="distance_controller",
        output='screen'
    )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': True}],
    # )

    return LaunchDescription([
        distance_controller_node,
        # rviz_node
    ])