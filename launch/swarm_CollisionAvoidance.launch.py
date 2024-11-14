import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher

node_frequency = 100 # [Hz]

def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_xyz.urdf')).read_text()
    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace=f'agent_{agent_id}',
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':f'agent_{agent_id}',
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return crazyflie_driver

def generate_webots_world_file(robots, source_filename, target_filename):
    with open(source_filename, 'r') as source_file:
        contents = source_file.read()

    with open(target_filename, 'w') as target_file:
        target_file.write(contents)

        for robot in robots:
            template_filename = os.path.join(os.path.dirname(source_filename), f'obj_{robot["type"]}.wbt')
            with open(template_filename, 'r') as template_file:
                template = template_file.read()
                template = template.replace('$NAME', robot["name"])
                template = template.replace('$X', str(robot["position"][0]))
                template = template.replace('$Y', str(robot["position"][1]))
                template = template.replace('$Z', str(robot["position"][2]))
                target_file.write(template)


def generate_launch_description():

    # number of agents
    N = 10

    # generate initial positions to evaluate initial takeoff
    P = np.zeros((N, 3))   
    P[0] = np.array([-4.5, -4, 0.015])
    P[1] = np.array([-2.5, -4, 0.015])
    P[2] = np.array([-3.5, -5, 0.015])
    P[3] = np.array([-4.5, -6, 0.015])
    P[4] = np.array([-2.5, -6, 0.015])

    P[5] = np.array([-8.5, -32, 0.015])
    P[6] = np.array([-6.5, -32, 0.015])
    P[7] = np.array([-7.5, -33, 0.015])
    P[8] = np.array([-8.5, -34, 0.015])
    P[9] = np.array([-6.5, -34, 0.015])


    # initialize launch description
    launch_description = []

    # Generate Webots world
    robots = [{
                'name': f'agent_{i}',
                'type': 'crazyflie', 
                'position': P[i, :].tolist(),
            } for i in range(N)]
    
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_formation_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

    # add executables for each robot
    for i in range(N):

        init_pos = P[i, :].tolist()

        # webots exec
        launch_description.append(get_webots_driver_cf(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':'agent_{}'.format(i)},
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_tracking_webots_guidance', 
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i, 
                'init_pos': init_pos,}]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_tracking_webots_controller', 
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{                    
                'agent_id': i,
                }]))

        # reference
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_tracking_webots_trajectory', 
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{                         
                'agent_id': i, 
                }]))

    return LaunchDescription(launch_description)
