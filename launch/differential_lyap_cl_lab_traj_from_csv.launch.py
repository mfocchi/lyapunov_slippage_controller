import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime
from launch.actions import ExecuteProcess
import csv

def generate_launch_description():
    ld = LaunchDescription()
    package_share_dir = get_package_share_directory('unicycle_controller')
    filename_trajectory = 'optimal_traj_3_fine_13_10.csv'
    path_csv = os.path.join(package_share_dir,'config/',filename_trajectory)
    print(path_csv)
    origin_RF = [1.0,0.0,0.0,
                 0.0,0.0,-1.0,
                 0.0,1.0,0.0]
    # Read data from csv
    t_vec = []
    v_vec = []
    omega_vec = [] 
    x_vec = []
    y_vec = []
    theta_vec = []
    with open(path_csv, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        # Iterate through the rows in the CSV file
        for row in csv_reader:
            # Access columns by their names and convert values to numbers
            t_vec.append(float(row['t']))
            v_vec.append(float(row['v']))
            omega_vec.append(float(row['omega']))
            x_vec.append(float(row['x']))
            y_vec.append(float(row['y']))
            theta_vec.append(float(row['theta']))

    path_gen_dt = t_vec[1] - t_vec[0]
    # Add stop condition in the end
    v_vec.append(0.0)
    omega_vec.append(0.0)
    x_vec.append(x_vec[-1])
    y_vec.append(y_vec[-1])
    theta_vec.append(theta_vec[-1])

    # to be set manually prior the run via MOCAP
    # ros2 launch unicycle_controller test_optitrack.launch
    pose_init = [1.172, -0.348, -2.942] 

    optitrack_node = Node(
        package="optitrack_interface",
        executable="optitrack",
        name="optitrack",
    )
    
    controller_node = Node(
        package="unicycle_controller",
        executable="differential_drive_cl_node",
        name="ctrl",
        output='screen',
        parameters=[
            {"enable_coppeliasim": False},
            {"Kp": 0.1},
            {"Ktheta": 2.0},
            {"dt": path_gen_dt},
            {"pub_dt_ms": 10},
            {"v_des_mps" : v_vec},
            {"omega_des_radps" : omega_vec},
            {"x_des_m" : x_vec},
            {"y_des_m" : y_vec},
            {"theta_des_rad" : theta_vec},
            {"pose_init_m_m_rad" : pose_init},
            {"origin_RF" : origin_RF},
            {"wheel_radius_m": 0.0856},
            {"wheels_distance_m": 0.606},
            {'gearbox_ratio': 34.45},
            {'copy_trajectory': True},
            {'automatic_pose_init': True},
            {'time_for_pose_init_s': 1.0},
        ]
    )
    robot_node = Node(
        package="maxxii_interface",
        executable="maxxii_node",
        name="maxxii_node",
        output='screen'
    )

    now = datetime.now()

    dt_string = now.strftime("%d-%m-%H-%M-%S")
    bag_string = 'bagfiles/cl_opt_'
    bag_name = bag_string + dt_string + '.bag'
    record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o%s' %bag_name]
    )
    
    ld.add_action(optitrack_node)
    ld.add_action(controller_node)
    ld.add_action(robot_node)
    ld.add_action(record_node)
    return ld
