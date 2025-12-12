import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Get the package share directory
    my_robot_description_pkg = get_package_share_directory('my_robot_description')
    my_robot_bringup_pkg = get_package_share_directory('my_robot_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')
    urdf_path = os.path.join(my_robot_description_pkg, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(my_robot_bringup_pkg, 'rviz', 'urdf_config.rviz')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo launch (forward world argument)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    # Spawn entity (with timeout increased to 120 seconds for large worlds)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-timeout', '120'],
        output='screen'
    )

    # RViz2 launch
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('world', default_value=os.path.join(my_robot_description_pkg, 'world', 'warehouse.world'),
                              description='Full path to world file'),
        robot_state_publisher_node,
        gazebo,
        spawn_entity_node,
        rviz2_node
    ])
