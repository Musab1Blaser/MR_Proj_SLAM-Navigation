import os
import random

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

from launch_ros.actions import Node

robo_spawn_list = [(2.5, -1.5, 1.57), (6.5, -6, 1.57/2), (-6.5, -5, 1.57*3/2), (-6, 6, 1.57*5/2), (7, 5, 1.57*7/2)]

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='differential_drive_robot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # worlds: office, construction, maze
    worlds = ['slam_maps/gazebo_models_worlds_collection-master/worlds/office_earthquake.world', 'slam_maps/gazebo_models_worlds_collection-master/worlds/office_cpr_construction.world', 'small_maze/smaze2d.world']
    # Path to the world file (replace with your actual world file path)
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', worlds[0])
    print(world_file_path) # for debugging

    # Include the Gazebo launch file with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    robot_spawn_point = random.choice(robo_spawn_list)
    print(robot_spawn_point)
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'differential_drive_robot',
                                        '-x', str(robot_spawn_point[0]),  # Set x-coordinate
                                        '-y', str(robot_spawn_point[1]),  # Set y-coordinate
                                        '-z', '0.2',  # Set z-coordinate
                                        '-Y', str(robot_spawn_point[2])  # Set yaw (e.g., 90 degrees in radians)
                                    ],
                        output='screen')
    
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', 'worlds/slam_maps/gazebo_models_worlds_collection-master/models/human_male_1/model.sdf',
            '-entity', 'target_person',
            '-x', '0.0', '-y', '0.0', '-z', '0.5', '-R', '1.57'
        ],
        output='screen'
    )

    targetX = 0
    targetY = 0

    controller_node = Node(
        package= 'differential_drive_robot',
        executable='lidar-nav.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'online_async_launch.py')]),
        launch_arguments={'params_file': 'config/mapper_params_online_async.yaml', 'use_sim_time': 'true'}.items()
    )


    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'navigation_launch.py')]),
        launch_arguments={'params_file': 'config/nav2_params.yaml', 'use_sim_time': 'true'}.items()
    )

    # TimerAction to delay the navigation node
    # delayed_navigation = TimerAction(
    #     period=5.0,  # Delay in seconds
    #     actions=[navigation]
    # )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        spawn_target,
        slam,
        navigation,
        controller_node,
    ])