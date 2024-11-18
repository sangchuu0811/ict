from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import rclpy
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route
    inspection_route = [
        [5.500, -2.0], [9.500, -2.0]
    ]

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = -4.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Prepare waypoints
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.pose.orientation.z = 1.0
    inspection_pose.pose.orientation.w = 0.0
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(inspection_pose))

    # Subscribe to input topic for InputAtWaypoint
    input_received = False

    def input_callback(msg):
        nonlocal input_received
        input_received = True
        print("Input received, proceeding to the next waypoint...")

    input_node = rclpy.create_node('input_listener')
    input_subscriber = input_node.create_subscription(Empty, 'input_at_waypoint/input', input_callback, 10)

    # Loop through waypoints
    for idx in range(len(inspection_points)):
        waypoint = inspection_points[idx]
        print(f"Moving to waypoint {idx + 1}/{len(inspection_points)}...")
        navigator.goToPose(waypoint)

        # Wait for the robot to reach the waypoint
        while not navigator.isTaskComplete():
            rclpy.spin_once(input_node, timeout_sec=0.1)  # Check for input while moving

        # At this point, the robot has reached the waypoint
        print(f'At waypoint {idx + 1}/{len(inspection_points)}. Waiting for input...')
        input_received = False  # Reset for the current waypoint

        # Wait for input
        while not input_received:
            rclpy.spin_once(input_node, timeout_sec=0.1)  # Wait for input

        # Proceed to the next waypoint
        if idx + 1 < len(inspection_points):
            # 다음 경유지로 이동
            next_waypoint = inspection_points[idx + 1]
            print(f"Proceeding to waypoint {idx + 2}/{len(inspection_points)}...")
            navigator.goToPose(next_waypoint)  # 다음 경유지로 이동

    """
    for idx, waypoint in enumerate(inspection_points):
        print(f"Moving to waypoint {idx + 1}/{len(inspection_points)}...")
        navigator.goToPose(waypoint)

        # Wait for the robot to reach the waypoint
        while not navigator.isTaskComplete():
            rclpy.spin_once(input_node, timeout_sec=0.1)  # Check for input while moving

        # At this point, the robot has reached the waypoint
        print(f'At waypoint {idx + 1}/{len(inspection_points)}. Waiting for input...')
        input_received = False  # Reset for the current waypoint
        
        # Stop the robot while waiting for input
        navigator.cancelTask()  # 로봇을 멈춤

        # Wait for input
        while not input_received:
            rclpy.spin_once(input_node, timeout_sec=0.1)  # Wait for input

        # Resume navigation after receiving input
        navigator.goToPose(waypoint)  # 로봇을 재개
    """
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
        exit(1)
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')

    # Return to start
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # navigator.goToPose(initial_pose)
    # while not navigator.isTaskComplete():
    #     pass

    exit(0)

if __name__ == '__main__':
    main()
