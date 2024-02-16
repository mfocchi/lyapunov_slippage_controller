from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
from launch.actions import ExecuteProcess
from datetime import datetime


def generate_launch_description():
    np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
    ld = LaunchDescription()
    
    # dt = 0.005 # [s]
    # long_v = -0.0 # [m/s]
    # # turning_radius = -0.3 # [m]
    # # ang_w = long_v / turning_radius # [rad/s]
    # ang_w = 0.1
    # t_end = 20 # [s]
    # n = (np.ceil(t_end / dt)).astype(int)    
    # v_vec     = np.linspace(long_v,   long_v, n).tolist()
    # omega_vec = np.linspace(ang_w,   ang_w, n).tolist()
    # v_vec.append(0.0)
    # omega_vec.append(0.0)

    R_initial = 0.1
    R_final = 0.6
    dt = 0.005  # [s] 200Hz
    long_v = 0.1  # [m/s]
    change_percentage = 0.5
    turning_radius = np.arange(R_initial, R_final, 0.1)
    ang_w = np.round(long_v / turning_radius,3)  # [rad/s]
    omega_vec = []
    v_vec = []
    theta = 0
    i = 0
    while True:
        theta += abs(ang_w[i])*dt
        #print(theta)
        omega_vec.append(ang_w[i])
        v_vec.append(long_v)
        if (theta > (1+i)*(2*np.pi * change_percentage)):
            i +=1
        if i == len(turning_radius):
            break
    #print(omega_vec)
    #print(ang_w)
    v_vec.append(0.0)
    omega_vec.append(0.0)


    optitrack_node = model_conv_node = Node(
        package="optitrack_interface",
        executable="optitrack",
        name="optitrack",
        output='screen',
    )

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
    #param_string = 'v%1.0f_omega%1.0f_' % (100*v_vec[0], 100*omega_vec[0])
    param_string = 'long_v%1.0f_Rinit%1.0f_Rend%1.0f_' % (100*long_v, 100*R_initial, 100*R_final)
    bag_string = 'bagfiles/ol_'
    bag_name = bag_string + param_string + dt_string + '.bag'
    record_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
        )
    ld.add_action(optitrack_node)
    ld.add_action(controller_node)
    ld.add_action(robot_node)
    ld.add_action(record_node)
    return ld
