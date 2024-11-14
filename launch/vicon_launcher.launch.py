from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

import numpy as np

node_frequency = 100 # [Hz]

def generate_launch_description():

    # number of agents
    N = 4

    # Set uri address 
    uris = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://1/90/2M/E7E7E7E702',
        'radio://2/100/2M/E7E7E7E703',
        'radio://3/70/2M/E7E7E7E704',
    ]

    P = np.zeros((N, 3)) 
    # Swarm
    P[0] = np.array([-3, -1, 0.015])  
    P[1] = np.array([-2, -1, 0.015])
    P[2] = np.array([-3, -2, 0.015])
    P[3] = np.array([-2, -2, 0.015])

    #Circle
    # P[0] = np.array([-2, 0, 0.015])  
    # P[1] = np.array([0, 2, 0.015])
    # P[2] = np.array([2, 0, 0.015])
    # P[3] = np.array([0, -2, 0.015])

    # initialize launch description
    launch_description = []
    radio_launch = []

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_tracking_vicon_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))
            
    # Launch radio node
    radios = set([int(uri.split('/')[2]) for uri in uris])
    for r in radios:
        uris_r = [uri for uri in uris if int(uri.split('/')[2]) == r]
        agent_ids_r = [uris.index(uri_r) for uri_r in uris_r]
        radio_launch.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_tracking_vicon_radio', 
            output='screen',
            namespace='radio_{}'.format(r),
            prefix='xterm -title "radio_{}" -hold -e'.format(r),
            parameters=[{
                'uris': uris_r,
                'agent_ids': agent_ids_r,
                'cmd_topic': 'traj_params',
                }]))   


    for i in range(N):
        
        uri = uris[i]
        init_pos = P[i, :].tolist()

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_tracking_vicon_guidance', 
            prefix='xterm -title "guidance_{}" -hold -e'.format(i),
            output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'freq': node_frequency, 
                'agent_id': i,
                'N': N, 
                'vicon_id': uri[-1],
                'vicon': True,
                'init_pos': init_pos}]))

        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_tracking_vicon_controller', 
            namespace='agent_{}'.format(i),
            prefix='xterm -title "controller_{}" -hold -e'.format(i),
            output='screen',
            parameters=[{
                'freq': node_frequency,                       
                'vicon_id': uri[-1],
                'vicon': True,
                'agent_id': i,
                }]))

        # reference
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_formation_vicon_trajectory', 
            namespace='agent_{}'.format(i),
            output='screen',
            parameters=[{     
                'freq': node_frequency,                       
                'vicon': True,
                'vicon_id': uri[-1],
                'agent_id': i, 
                }]))


    # include delayed radio launcher
    timer_action = TimerAction(period=5.0, actions=[LaunchDescription(radio_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
