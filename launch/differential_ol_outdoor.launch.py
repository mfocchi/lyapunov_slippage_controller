from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
from launch.actions import ExecuteProcess
from datetime import datetime

# run qualisys with : ros2 launch qualisys_driver qualisys.launch.py

def generate_launch_description():
    ld = LaunchDescription()
    dt = 0.05
    long_v = 0.15
    ang_w = 0.3

    t_end = 20
    n = (np.ceil(t_end / dt)).astype(int)
    v_vec     = np.linspace(long_v,   long_v, n).tolist()
    omega_vec = np.linspace(ang_w,   ang_w, n).tolist()
    v_vec.append(0.0)
    omega_vec.append(0.0)

    controller_node = Node(
        package="lyapunov_slippage_controller",
        executable="differential_drive_ol_node",
        name="ctrl",
        output='screen',
        parameters=[
            {"enable_coppeliasim": False},
            {"enable_sim_render":  True},
            {"pub_dt_ms": int(dt*1000)},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"wheel_radius_m": 0.0856},
            {"wheels_distance_m": 0.606},
            {'gearbox_ratio': 34.45},
            {'copy_trajectory': False},
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
    bag_string = 'bagfiles/ol_outdooor'
    bag_name = bag_string + param_string + dt_string + '.bag'
    record_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
        )
    ld.add_action(controller_node)
    ld.add_action(robot_node)
    ld.add_action(record_node)
    return ld
