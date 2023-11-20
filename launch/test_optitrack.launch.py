from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    ld = LaunchDescription()
    origin_RF = [1.0,0.0,0.0,
                 0.0,0.0,-1.0,
                 0.0,1.0,0.0]
    
    optitrack_node = Node(
        package="optitrack_interface",
        executable="optitrack",
        name="optitrack",
    )
    
    test_node = Node(
        package="unicycle_controller",
        executable="feedbackTester",
        name="test",
        output='screen',
        parameters=[
            {"origin_RF" : origin_RF},
        ]
    )
    
    ld.add_action(optitrack_node)
    ld.add_action(test_node)
    return ld
