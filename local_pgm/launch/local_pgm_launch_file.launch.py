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

    BaseLinkPublisher = Node(
        package='local_pgm',
        executable='base_link_publisher',
        name='base_link_publisher',
        output='screen',
        emulate_tty=True,
    )

    odom_node = Node(
        package='local_pgm',
        executable='odom_node',
        name='odom_node',
        output='screen',
        emulate_tty=True,
        # parameters=[{'use_sim_time': True}]
    )

    FirebasePublisher = Node(
        package='local_pgm',
        executable='firebase_publisher',
        name='firebase_publisher',
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([
        # tf_broadcaster,
        # gps_receiver,
        In,
        BaseLinkPublisher,
        odom_node,
        # FirebasePublisher,
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
