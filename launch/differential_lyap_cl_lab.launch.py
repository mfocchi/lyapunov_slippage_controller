from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime
from launch.actions import ExecuteProcess
import numpy as np
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    origin_RF = [1.0,0.0,0.0,
                 0.0,0.0,-1.0,
                 0.0,1.0,0.0]
    dt = 0.01
    t_end = 5.0
    n = (np.ceil(t_end / dt)).astype(int)
    
    v_vec     = np.linspace(0.15,   0.15, n).tolist()
    omega_vec = np.linspace(0.15/0.7, 0.15/0.7, n).tolist()

    v_vec.append(0.0)
    omega_vec.append(0.0)

    pose_init = [0.587, -0.373, 3.031] # must be set if using a global localization system

    optitrack_node = Node(
        package="optitrack_interface",
        executable="optitrack",
        name="optitrack",
    )

    controller_node = Node(
        package="lyapunov_slippage_controller",
        executable="differential_drive_cl_node",
        name="ctrl",
        output='screen',
        parameters=[
            {"enable_coppeliasim": False},
            {"Kp": 1.0},
            {"Ktheta": 1.0},
            {"dt": dt},
            {"pub_dt_ms": 50},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"pose_init_m_m_rad" : pose_init},
            {"origin_RF" : origin_RF},
            {"wheel_radius_m": 0.0856},
            {"wheels_distance_m": 0.606},
            {'gearbox_ratio': 34.45},
            {'copy_trajectory': False},
            {'automatic_pose_init': True},
            {'time_for_pose_init_s': 0.5},
        ]
    )
    robot_node = Node(
        package="maxxii_interface",
        executable="maxxii_node",
        name="maxxii_node",
        output='screen'
    )

    now = datetime.now()

    dt_string = now.strftime("%d-%m-%H-%M")
    param_string = 'v%1.0f_omega%1.0f_' % (100*v_vec[0], 100*omega_vec[0])
    bag_string = 'bagfiles/cl_gains_'
    bag_name = bag_string + param_string + dt_string + '.bag'
    record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
    )
    
    ld.add_action(optitrack_node)
    ld.add_action(controller_node)
    ld.add_action(robot_node)
    ld.add_action(record_node)
    return ld
