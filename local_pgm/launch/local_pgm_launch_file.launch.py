from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tf_broadcaster = Node(
        package='local_pgm',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
        emulate_tty=True,
    )

    gps_receiver = Node(
        package='local_pgm',
        executable='gps_receiver',
        name='gps_receiver',
        output='screen',
        emulate_tty=True,
    )

    In = Node(
        package='local_pgm',
        executable='in',
        name='in',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # tf_broadcaster,
        # gps_receiver,
        In,
    ])
    # return LaunchDescription([
    #     Node(
    #         package='local_pgm',
    #         executable='tf_broadcaster',
    #         output='screen',
    #     ),
    #     Node(
    #         package='local_pgm',
    #         executable='gps_receiver',
    #         output='screen',
    #     ),
