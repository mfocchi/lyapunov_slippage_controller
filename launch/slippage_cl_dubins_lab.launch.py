import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from datetime import datetime
from launch.actions import ExecuteProcess
import numpy as np
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()


    #old way to generate dubins
    # experiment_id = 100
    # scale_factor = 2.0 # used to calculate the the scale factor
    # omega_dubins_vec = get_exp_omega_vec(experiment_id, scale_factor)
    # times_dubins_vec = get_exp_time_vec(experiment_id, scale_factor)
    # path_gen_dt = 0.005
    # omega_vec = extract_settings_from_dubins(omega_dubins_vec, times_dubins_vec, path_gen_dt)
    # n = len(omega_vec)
    # v_vec     = np.linspace(longitudinal_velocity,longitudinal_velocity, n).tolist()
    
    path_gen_dt = 0.005

    #override with fixed vel 
    longitudinal_velocity = 0.3
    angular_velocity = 0.5
    n = 1000
    v_vec     = np.linspace(longitudinal_velocity,longitudinal_velocity, n).tolist()
    omega_vec = np.linspace(angular_velocity,angular_velocity, n).tolist()
    # stop execution of control inputs
    v_vec.append(0.0)
    v_vec.append(0.0)
    v_vec.append(0.0)
    omega_vec.append(0.0)
    omega_vec.append(0.0)
    omega_vec.append(0.0)

    pose_init = [0.0,0.0,0.0]

    optitrack_node = Node(
        package="optitrack_interface",
        executable="optitrack",
        name="optitrack",
    )

    Simulation = True
    RosBagRecord = False

    Kp = 5.
    Kth = 10.0
    controller_node = Node(
        package="lyapunov_slippage_controller",
        executable="slippage_controller_node",
        name="ctrl",
        output='screen',
        emulate_tty=True,
        parameters=[
            {"wheel_radius_m": 0.0856},
            {"wheels_distance_m": 0.606},
            {"enable_coppeliasim": Simulation},
            {"Kp": Kp},
            {"Ktheta": Kth},
            {"dt": path_gen_dt},
            {"pub_dt_ms": 100},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"pose_init_m_m_rad" : pose_init},
            {'automatic_pose_init': True},
            {'time_for_pose_init_s': 1.0},
            {'side_slip_angle_coefficients_left': [-0.3465,   -3.2004]}, #this are for left turn positive radius
            {'side_slip_angle_coefficients_right': [0.3465 ,  3.2004]}, #this are for right turn negative radius
            {'beta_slip_outer_coefficients_left': [ 0.0473 ,  -1.8081]},#this are for right turn negative radius
            {'beta_slip_outer_coefficients_right': [ 0.0857,    1.8]},#this are for left turn positive radius
            {'beta_slip_inner_coefficients_left': [ 0.0532 ,  -2.2327]},#this are for left turn positive radius
            {'beta_slip_inner_coefficients_right': [0.0502 ,   2.3925]},#this are for right turn negative radius
            {'consider_slippage': False},
            {'planner_type': "optim"},#dubins/optim
        ],
        on_exit=launch.actions.Shutdown(),
    )
    if not Simulation:
        robot_node = Node(
            package="ros2_maxxii", #old driver was maxxii_interface
            executable="maxxii_node",
            name="maxxii_node",
            output='screen'
        )
        ld.add_action(optitrack_node)
        ld.add_action(robot_node)
    else:
        #kill previous instances 
        os.system("pkill coppeliaSim")
        os.system("ros2  lifecycle set  /sim_ros2_interface shutdown")
        os.system("ros2  lifecycle set  /transform_listener shutdown")
        scene = get_package_share_directory('lyapunov_slippage_controller')+'/config/tractor_ros2.ttt'
        file = os.getenv("COPPELIASIM_ROOT_DIR")+"/coppeliaSim.sh"+" "+scene+" &"
        os.system(file)

    now = datetime.now()

    dt_string = now.strftime("%d-%m-%H-%M-%S")
    param_string = 'exp_'
    bag_string = 'bagfiles/slip_test_'
    bag_name = bag_string + param_string + dt_string + '.bag'
    record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
    )
    
    ld.add_action(controller_node)
    if RosBagRecord:
        ld.add_action(record_node)
    return ld

def extract_settings_from_dubins(omega_vec, time_vec, dt):
    # omega_vec, since it represents a Dubins path, can have
    # maximum 3 values. Each value represents how the robot 
    # behaves in the time interval (left turn, right turn, straight)
    # time vec has maximum 4 values and represents the time 
    # interval edges. This function extract the desired omega_vec
    # for each integration step
    omega_fine_vec = []
    for i in range(len(omega_vec)):
        omega_motion = omega_vec[i]
        n = int((time_vec[i+1] - time_vec[i]) / dt)
        omegas_tmp = np.linspace(omega_motion, omega_motion, n).tolist()
        omega_fine_vec += omegas_tmp
    return omega_fine_vec

def get_exp_omega_vec(exp_id, scale_factor):
    if(exp_id == 1):
        return (np.array([-0.2000, 0, 0.2000])*scale_factor).tolist() 
    elif(exp_id == 2):
        return (np.array([-0.2000, 0, -0.2000])*scale_factor).tolist()
    elif(exp_id == 3):
        return (np.array([-0.2000, 0, -0.2000])*scale_factor).tolist()
    elif(exp_id == 4):
        return (np.array([-0.2000, 0, -0.2000])*scale_factor).tolist()
    elif(exp_id == 5):
        return (np.array([-0.2000, 0, 0.2000])*scale_factor).tolist()
    elif(exp_id == 6):
        return (np.array([0.1786, 0, 0.1786])*scale_factor).tolist()
    elif(exp_id == 7):
        return (np.array([-0.1786, 0, -0.1786])*scale_factor).tolist()
    elif(exp_id == 8):
        return (np.array([-0.1786, 0, -0.1786])*scale_factor).tolist()
    elif(exp_id == 9):
        return (np.array([-0.1786, 0, -0.1786])*scale_factor).tolist()
    elif(exp_id == 10):
        return (np.array([-0.1786, 0, 0.1786])*scale_factor).tolist()
    elif(exp_id == 11):
        return (np.array([0.1667, 0, 0.1667])*scale_factor).tolist()
    elif(exp_id == 12):
        return (np.array([-0.1667, 0, -0.1667])*scale_factor).tolist()
    elif(exp_id == 13):
        return (np.array([-0.1667, 0, -0.1667])*scale_factor).tolist()
    elif(exp_id == 14):
        return (np.array([0.1667, 0, -0.1667])*scale_factor).tolist()
    elif(exp_id == 15):
        return (np.array([-0.1667, 0, 0.1667])*scale_factor).tolist()
    elif(exp_id == 16):
        return (np.array([0.1562, 0, 0.1562])*scale_factor).tolist()
    elif(exp_id == 17):
        return (np.array([-0.1562, 0, -0.1562])*scale_factor).tolist()
    elif(exp_id == 18):
        return (np.array([0.1562, -0.1562, 0.1562])*scale_factor).tolist()
    elif(exp_id == 19):
        return (np.array([0.1562, 0, -0.1562])*scale_factor).tolist()
    elif(exp_id == 20):
        return (np.array([-0.1562, 0, 0.1562])*scale_factor).tolist()
    elif(exp_id == 21):
        return (np.array([0.1429, 0, 0.1429])*scale_factor).tolist()
    elif(exp_id == 22):
        return (np.array([-0.1429, 0, -0.1429])*scale_factor).tolist()
    elif(exp_id == 23):
        return (np.array([0.1429, -0.1429, 0.1429])*scale_factor).tolist()
    elif(exp_id == 24):
        return (np.array([0.1429, 0, -0.1429])*scale_factor).tolist()
    elif(exp_id == 25):
        return (np.array([-0.1429, 0, 0.1429])*scale_factor).tolist()
    elif(exp_id == 26):
        return (np.array([0.1250, 0, 0.1250])*scale_factor).tolist()
    elif(exp_id == 27):
        return (np.array([-0.1250, 0, -0.1250])*scale_factor).tolist()
    elif(exp_id == 28):
        return (np.array([0.1250, -0.1250, 0.1250])*scale_factor).tolist()
    elif(exp_id == 29):
        return (np.array([0.1250, 0, -0.1250])*scale_factor).tolist()
    elif(exp_id == 30):
        return (np.array([-0.1250, 0.1250, -0.1250])*scale_factor).tolist()
    elif(exp_id == 31):
        return (np.array([0.1111, 0, 0.1111])*scale_factor).tolist()
    elif(exp_id == 32):
        return (np.array([-0.1111, 0, -0.1111])*scale_factor).tolist()
    elif(exp_id == 33):
        return (np.array([-0.1111, 0.1111, -0.1111])*scale_factor).tolist()
    elif(exp_id == 34):
        return (np.array([0.1111, 0, -0.1111])*scale_factor).tolist()
    elif(exp_id == 35):
        return (np.array([-0.1111, 0.1111, -0.1111])*scale_factor).tolist()
    elif(exp_id == 100): # special case with one constant curve
        return (np.array([-0.1111, -0.1111, -0.1111])*scale_factor).tolist()
    elif(exp_id == -1): 
        return [0.5, 0, 0.5]
    else:
        print("ID for experiment selection is not in range")
        return (np.array([0.0,0.0,0.0])*scale_factor).tolist()
    
def get_exp_time_vec(exp_id, scale_factor):
    if(exp_id == 1):
        return (np.array([0.0, 19.8656, 28.0567, 45.3838])/scale_factor).tolist()
    elif(exp_id == 2):
        return (np.array([0.0, 9.0934, 29.1615, 45.6651])/scale_factor).tolist()
    elif(exp_id == 3):
        return (np.array([0.0, 5.6814, 7.7114, 11.3149])/scale_factor).tolist()
    elif(exp_id == 4):
        return (np.array([0.0, 0.3347, 16.6246, 35.8269])/scale_factor).tolist()
    elif(exp_id == 5):
        return (np.array([0.0, 2.8880, 12.3754, 31.8318])/scale_factor).tolist()
    elif(exp_id == 6):
        return (np.array([0.0, 21.5474, 38.1293, 48.9246])/scale_factor).tolist()
    elif(exp_id == 7):
        return (np.array([0.0, 10.0573, 29.6439, 48.2553])/scale_factor).tolist()
    elif(exp_id == 8):
        return (np.array([0.0, 7.3706, 8.4783, 11.5068])/scale_factor).tolist()
    elif(exp_id == 9):
        return (np.array([0.0, 0.0192, 16.6885, 38.5508])/scale_factor).tolist()
    elif(exp_id == 10):
        return (np.array([0.0, 3.9496, 12.1589, 34.6651])/scale_factor).tolist()
    elif(exp_id == 11):
        return (np.array([0.0, 23.1464, 39.6140, 51.1204])/scale_factor).tolist()
    elif(exp_id == 12):
        return (np.array([0.0, 10.6810, 29.9524, 49.9879])/scale_factor).tolist()
    elif(exp_id == 13):
        return (np.array([0.0, 10.5359, 11.1047, 11.7107])/scale_factor).tolist()
    elif(exp_id == 14):
        return (np.array([0.0, 0.2264, 16.7320, 40.4028])/scale_factor).tolist()
    elif(exp_id == 15):
        return (np.array([0.0, 4.8173, 11.9395, 36.6388])/scale_factor).tolist()
    elif(exp_id == 16):
        return (np.array([0.0, 24.7544, 41.1092, 53.3178])/scale_factor).tolist()
    elif(exp_id == 17):
        return (np.array([0.0, 11.2888, 30.2499, 51.7254])/scale_factor).tolist()
    elif(exp_id == 18):
        return (np.array([0.0, 8.1929, 36.7281, 45.1858])/scale_factor).tolist()
    elif(exp_id == 19):
        return (np.array([0.0, 0.5041, 16.7712, 42.2826])/scale_factor).tolist()
    elif(exp_id == 20):
        return (np.array([0.0, 5.8789, 11.6051, 38.6915])/scale_factor).tolist()
    elif(exp_id == 21):
        return (np.array([0.0, 27.1833, 43.3722, 56.6172])/scale_factor).tolist()
    elif(exp_id == 22):
        return (np.array([0.0, 12.1688, 30.6746, 54.3416])/scale_factor).tolist()
    elif(exp_id == 23):
        return (np.array([0.0, 9.3380, 41.2895, 50.9040])/scale_factor).tolist()
    elif(exp_id == 24):
        return (np.array([0.0, 0.9820, 16.8197, 45.1535])/scale_factor).tolist()
    elif(exp_id == 25):
        return (np.array([0.0, 8.4519, 10.3232, 41.9709])/scale_factor).tolist()
    elif(exp_id == 26):
        return (np.array([0.0, 31.2785, 47.1997, 62.1250])/scale_factor).tolist()
    elif(exp_id == 27):
        return (np.array([0.0, 13.5450, 31.3201, 58.7304])/scale_factor).tolist()
    elif(exp_id == 28):
        return (np.array([0.0, 11.2103, 48.7807, 60.2848])/scale_factor).tolist()
    elif(exp_id == 29):
        return (np.array([0.0, 1.9505, 16.8618, 50.0715])/scale_factor).tolist()
    elif(exp_id == 30):
        return (np.array([0.0, 10.7391, 48.8406, 49.6937])/scale_factor).tolist()
    elif(exp_id == 31):
        return (np.array([0.0, 35.4345, 51.0996, 67.6444])/scale_factor).tolist()
    elif(exp_id == 32):
        return (np.array([0.0, 14.7966, 31.8806, 63.1587])/scale_factor).tolist()
    elif(exp_id == 33):
        return (np.array([0.0, 6.3935, 60.7169, 68.8110])/scale_factor).tolist()
    elif(exp_id == 34):
        return (np.array([0.0, 3.1564, 16.8304, 55.1535])/scale_factor).tolist()
    elif(exp_id == 35):
        return (np.array([0.0, 12.0166, 55.6281, 57.4000])/scale_factor).tolist()
    elif(exp_id == 100): # special case with one constant curve
        return (np.array([0.0, 12.0166, 55.6281, 57.4000])/scale_factor).tolist()
    elif(exp_id == -1): 
        return [0.0, 1.53204654144835,8.12756557496897,9.74385138162452]
    else:
        print("ID for experiment selection is not in range")
        return (np.array([0.0, 0.0, 0.0, 0.0])/scale_factor).tolist()