import os
import random

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

from launch_ros.actions import Node

robo_spawn_list = [(2.5, -1.5, 1.57), (6.5, -6, 1.57/2), (-6.5, -5, 1.57*3/2), (-6, 6, 1.57*5/2), (7, 5, 1.57*7/2)]
use_rand_spawning = False

def generate_launch_description():

    package_name='differential_drive_robot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # worlds: office, construction, maze
    worlds = ['slam_maps/gazebo_models_worlds_collection-master/worlds/office_earthquake.world', 
            #   'slam_maps/gazebo_models_worlds_collection-master/worlds/office_cpr_construction.world', 'small_maze/smaze2d.world'
              ]
    # Path to the world file (replace with your actual world file path)
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', worlds[0])
    print(world_file_path) # for debugging

    # Include the Gazebo launch file with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    robot_spawn_point = random.choice(robo_spawn_list)
    if not use_rand_spawning:
        robot_spawn_point = (3, -4, 0) # Specify custom robot position if not using random
    # robot_spawn_point = robo_spawn_list[1]

    print(robot_spawn_point)
    
    # Run the spawner node from the gazebo_ros package. 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'differential_drive_robot',
                                        '-x', str(robot_spawn_point[0]),  # Set x-coordinate
                                        '-y', str(robot_spawn_point[1]),  # Set y-coordinate
                                        '-z', '0.2',  # Set z-coordinate
                                        '-Y', str(robot_spawn_point[2])  # Set yaw (e.g., 90 degrees in radians)
                                    ],
                        output='screen')
    


    # Define the boundaries of the environment
    x_min, x_max = -8, 8
    y_min, y_max = -8, 8

    # Define the rectangular regions to avoid
    obstacle_regions = [
        (-7, -4, 7, 10),      # Adjusted Top-Left Region
        (-2.5, 2.5, -3.5, 4.5), # Adjusted Centre Region
        (4, 10, 8, 10),        # Adjusted Top-Right Region
        (8, 10, 4, 10),         # Adjusted Middle-Right Region
        (-10, -8, 2, -10),     # Adjusted Bottom-Left Region
        (-10, 5, -8, -10),  # Adjusted Bottom-Centre Region
        (6, 9, -8, -3),       # Adjusted Bottom-Right Region
    ]

    def is_in_obstacle(x, y, obstacles):
        """Check if a point (x, y) is in any obstacle region."""
        for region in obstacles:
            if region[0] <= x <= region[1] and region[2] <= y <= region[3]:
                return True
        return False

    def generate_random_goal(obstacles, x_bounds, y_bounds):
        """Generate random coordinates avoiding obstacles."""
        while True:
            # Generate random x and y within the bounds
            x = random.uniform(x_bounds[0], x_bounds[1])
            y = random.uniform(y_bounds[0], y_bounds[1])
            
            # Check if the point is in any obstacle region
            if not is_in_obstacle(x, y, obstacles):
                return x, y
            

    # Generate a random goal point
    random_goal = generate_random_goal(obstacle_regions, (x_min, x_max), (y_min, y_max))
    print(f"Random goal coordinates: {random_goal}")
    targetX, targetY = random_goal
    if not use_rand_spawning:
        targetX, targetY = 5.5, 3.5 # Specify custom target position if not using random
    targetW = 1

    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', 'worlds/slam_maps/gazebo_models_worlds_collection-master/models/human_male_1/model.sdf',
            '-entity', 'target_person',
            '-x', str(targetX), '-y', str(targetY), '-z', '0.1', '-R', '1.57'
        ],
        output='screen'
    )

    # Write target coordinates to goal.txt
    goal_file_path = os.path.join(get_package_share_directory(package_name), 'goal.txt')
    print("goal file:", goal_file_path) # for debugging
    with open(goal_file_path, 'w') as file:
        file.write(f"{targetX}\n{targetY}\n{targetW}\n")

    controller_node = Node(
        package= 'differential_drive_robot',
        executable='publish_goal_pose.py',
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