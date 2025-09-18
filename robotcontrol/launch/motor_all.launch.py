
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    lib_path = os.path.join(os.getcwd(), 'src/robotcontrol/lib/bin/unix64/release/')
    env = {'LD_LIBRARY_PATH': lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')}
    return LaunchDescription([
        Node(
            package='robotcontrol',
            executable='motor_send_node',
            name='motor_send_node',
            output='screen',
            additional_env=env
        ),
        # TimerAction(
        #     period=5.0,  # 延迟5秒启动
        #     actions=[
        #         Node(
        #             package='robotcontrol',
        #             executable='motor_test_node',
        #             name='motor_test_node',
        #             output='screen',
        #             parameters=[{'motor_id': 1, 'current': 1000}],
        #             additional_env=env
        #         )
        #     ]
        # ),
        # 如需加PID节点，也建议用TimerAction延迟启动
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robotcontrol',
                    executable='motor_pid_node',
                    name='motor_pid_node',
                    output='screen',
                    parameters=[{'motor_id': 1, 'target_value': 1000}],
                    additional_env=env
                )
            ]
        ),
    ])
