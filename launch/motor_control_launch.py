import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
        )

    motor_front_right_node=Node(
        package = 'motor_control',
        #name = 'front_right_motor_control',
        executable = 'motor_control',
        parameters = [config],
        arguments=[('--ros-args --log-level debug')]
    )

    """motor_front_left_node=Node(
        package = 'motor_control',
        name = 'front_left_motor_control',
        executable = 'motor_control',
        parameters = [config]
    )"""


    '''motor_rear_right_node=Node(
        package = 'motor_control_py',
        name = 'rear_right_motor_control',
        executable = 'motor_controller',
        parameters = [config]
    )
    

    motor_rear_left_node=Node(
        package = 'motor_control_py',
        name = 'rear_left_motor_control',
        executable = 'motor_controller',
        parameters = [config]
    )'''

    ld.add_action(motor_front_right_node)
    #ld.add_action(motor_front_left_node)
    #ld.add_action(motor_rear_right_node)
    #ld.add_action(motor_rear_left_node)
    return ld