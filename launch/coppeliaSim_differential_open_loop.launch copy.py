from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    ld = LaunchDescription()
    dt = 0.1
    t_end = 60.0
    n = (np.ceil(t_end / dt)).astype(int)
    #v_vec     = np.linspace(0.1,  0.1, n).tolist()
    #omega_vec = np.linspace(0.01, 0.3, n).tolist()

    v_vec     = np.linspace(0.1,   0.8, n).tolist()
    omega_vec = np.linspace(0.1,   0.8, n).tolist()
    v_vec.append(0.0)
    omega_vec.append(0.0)

    pose_init = [0.0, 0.0, 0.0]

    model_conv_node = Node(
        package="unicycle",
        executable="unicycle_2_differential",
        name="unicycle_2_differential",
        
        parameters=[
            {"wheel_radius_m": 0.085},
            {"wheels_distance_m": 0.62}
        ]
    )
    controller_node = Node(
        package="unicycle_controller",
        executable="openLoopNode",
        name="ctrl",
        output='screen',
        parameters=[
            {"enable_coppeliasim": True},
            {"enable_sim_render":  True},
            {"start_wait_time_ms": 100},
            {"start_timeout_ms": 300},
            {"pub_dt_ms": int(dt*1000)},
            {"sim_dt_ms": 100},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
        ]
    )
    ld.add_action(model_conv_node)
    ld.add_action(controller_node)
    return ld
