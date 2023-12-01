from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    ld = LaunchDescription()

    v_vec = [0.2] * 199
    omega_vec = [0.2] * 199
    pose_init = [0.0, 0.0, np.pi / 2]

    model_sim_node = Node(
        package="robot_model",
        executable="unicycle_sim_node",
        name="unicycle_sim_node",
        
        parameters=[
            {"pub_dt_ms": 20},
            {"x0_m": 0.0002},
	        {"y0_m": -0.0003},
            {"theta0_rad": 0.0}
        ]
    )
    controller_node = Node(
        package="lyapunov_slippage_controller",
        executable="lyapunovControllerNode",
        name="ctrl",
        parameters=[
            {"enable_coppeliasim": False},
            {"enable_sim_render":  False},
            {"Kp": 1.0},
            {"Ktheta": 0.5},
            {"dt": 0.1},
            {"end_th": 0.01},
            {"pub_dt_ms": 200},
            {"sim_dt_ms": 100},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"pose_init_m_m_rad" : pose_init}
        ]
    )
    ld.add_action(model_sim_node)
    ld.add_action(controller_node)
    return ld