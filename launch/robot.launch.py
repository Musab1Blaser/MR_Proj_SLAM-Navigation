import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



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
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', worlds[2])
    print(world_file_path) # for debugging

    # Include the Gazebo launch file with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'differential_drive_robot',
                                        '-x', '5',  # Set x-coordinate
                                        '-y', '10',  # Set y-coordinate
                                        '-z', '0.0',  # Set z-coordinate
                                        '-Y', '1.57'  # Set yaw (e.g., 90 degrees in radians)
                                    ],
                        output='screen')

    controller_node = Node(
        package= 'differential_drive_robot',
        executable='open-loop.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        controller_node,
    ])