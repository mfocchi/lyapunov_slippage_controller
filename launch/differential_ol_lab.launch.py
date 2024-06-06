from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
from launch.actions import ExecuteProcess
from datetime import datetime
import math
from  termcolor import colored


def generate_launch_description():
    np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
    ld = LaunchDescription()

    omega_vec = []
    v_vec = []
    wheel_l_vec = []
    wheel_r_vec = []

    ident_type='wheels'# "v_omega"
    #########################
    #fixed
    ###########################
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

   
    if ident_type=='v_omega':
        ##################################
        #variable radius of curvature (change with time)
        #######################################
        #max speed of wheels (motors) is 1500 rpm and 157 rad /s => max omega is 1
        #R = [0:0.1: 0.4]; in matlab with coppeliasim with only turning left
        R_initial = 0.1    # THE MIMINUM ACHIEVABLE RADIUS ON REAL ROBOT IS 0.1
        R_final = 0.325    # it was = 0.6
        turning='left'
        dt = 0.005  # [s] 200Hz    -- the same as CoppeliaSim
        long_v = 0.15  # [m/s]   #0.05:0.025:0.15
        change_interval = 6.
        increment = 0.025       # it was = 0.05
        turning_radius = np.arange(R_initial, R_final, increment)
        # turning_radius_2 = np.append(turning_radius , np.arange(R_initial+increment/2, R_final-increment/2, increment))
        # turning_radius_3 = np.append(turning_radius_2 , np.arange(R_initial+increment/3, R_final-2*increment/3, increment))
        turning_radius_vec = turning_radius

        #turning left
        if turning=='left':
            ang_w = np.round(long_v / turning_radius_vec, 3)  # [rad/s]
        else:
        #turning right
            ang_w = -np.round(long_v / turning_radius_vec, 3)  # [rad/s]
     
        time = 0
        i = 0
        while True:
            time = np.round(time +dt,3)
            omega_vec.append(ang_w[i])
            v_vec.append(long_v)
            wheel_l_vec.append(0.0)
            wheel_r_vec.append(0.0)
            #detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1+i)*change_interval):
                i +=1
            if i == len(turning_radius_vec):
                break
        v_vec.append(0.0)
        omega_vec.append(0.0)
        #needs to be filled in otherwise ros2 complains
        wheel_l_vec.append(0.0)
        wheel_r_vec.append(0.0)

        bag_string = 'bagfiles/ol_'
        param_string = 'long_v%1.0f_Rinit%1.1f_Rend%1.1f_' % (100*long_v, 100*R_initial, 100*R_final)
        bag_name = bag_string + param_string +  turning + '.bag'

    if ident_type=='wheels':
        ####################################
        #OPEN LOOP wl wr (from -157 to 157)
        ####################################
        turning ='left'
        wheel_l = -160. #-160:20: 160 //these are the wheel rad/s at MOTOR SIDE
        change_interval = 2.
        increment = 40   # it was = 40
        dt = 0.005  # [s] 200Hz    -- the same as CoppeliaSim

        if wheel_l>=0:
            wheel_r = np.arange(160., -160.-increment, -increment)  
        else:     
            wheel_r = np.arange(-160., 160.+increment, increment)       
        time = 0
        i = 0
  
        while True:
            time = np.round(time+dt,3)
            wheel_l_vec.append(wheel_l)
            wheel_r_vec.append(wheel_r[i])
            v_vec.append(0.0)
            omega_vec.append(0.0)
            #detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1+i)*change_interval):
                i +=1
            if i == len(wheel_r):
                break
        
        wheel_l_vec.append(0.0)
        wheel_r_vec.append(0.0)
        #needs to be filled in otherwise ros2 complains
        v_vec.append(0.0)
        omega_vec.append(0.0)

        bag_string = 'bagfiles/ol_'
        param_string = 'wheel_l_%1.0f'%(wheel_l)
        bag_name = bag_string + param_string + '.bag'


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
            {"ident_type": ident_type},
            {"enable_coppeliasim": False},
            {"enable_sim_render":  True},
            {"pub_dt_ms": int(dt*1000)},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"wheel_l_vec" : wheel_l_vec},
            {"wheel_r_vec" : wheel_r_vec},
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

    #dt_string = now.strftime("%d-%m-%H-%M")
    record_node = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
        )

    print(colored("IMPORTANT if you are saving bags with the same name remove the previous bag!","red"))
    ld.add_action(optitrack_node)
    ld.add_action(controller_node)
    ld.add_action(robot_node)
    ld.add_action(record_node)
    return ld
