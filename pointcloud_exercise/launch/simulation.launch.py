import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Correct the package name to pointcloud_exercise
    world_path = os.path.join(
        get_package_share_directory('pointcloud_exercise'),
        'worlds',
        'simulation.world'
    )
    
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_path}.items(),
    )

    # Get the path to the URDF file
    simulation_urdf_path = os.path.join(
        get_package_share_directory('pointcloud_exercise'),
        'urdf',
        'camera.urdf'
    )

    # Load the robot description (URDF) content
    with open(simulation_urdf_path, 'r') as urdf_file:
        robot_description = {'robot_description': urdf_file.read()}

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', simulation_urdf_path,
            '-entity', 'camera',
            '-z', '1',
            '-P', '1.57'
        ],
        output='screen'
    )

    # Node to publish the robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Return a LaunchDescription containing all nodes
    return LaunchDescription([
        gazebo,
        spawn_entity,
        node_robot_state_publisher
    ])
