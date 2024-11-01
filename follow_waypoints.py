import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Twist, PoseStamped
from firebase_admin import db, credentials
import firebase_admin
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy

class CoordinateMarker:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

def parse_coordinates(data):
    data = data.strip("[]").split(", ")
    coordinates = []

    for item in data:
        lat_lon = item.split(" ")
        lat = float(lat_lon[0])
        lon = float(lat_lon[1])
        coordinates.append((lat, lon))

    return coordinates

class Waypoints(Node):
    def __init__(self):
        super().__init__('setDBwaypoint')

        # Firebase 초기화
        cred = credentials.Certificate("/home/hgy/ict/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
        })
        self.ref = db.reference('admin')

        # 퍼블리셔 설정
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 30)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker', 30)
        self.timer = self.create_timer(5.0, self.publish_data_from_firebase)

        self.marker_array = MarkerArray()
        self.navigator = BasicNavigator()
        self.robot_route = []

        # 초기 위치 설정
        self.set_initial_pose()

    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = -4.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        
        self.navigator.setInitialPose(initial_pose)

    def publish_data_from_firebase(self):
        marker_data = self.ref.child('marker/waypoint').get()

        if marker_data:
            coordinates = parse_coordinates(marker_data)
            self.update_visualization_markers(coordinates)

            if coordinates:
                self.robot_route = coordinates  # Firebase에서 가져온 좌표를 경로로 설정
                self.navigate_route()

    def update_visualization_markers(self, coordinates):
        self.marker_array.markers.clear()

        for lat, lon in coordinates:
            visual_marker = Marker()
            visual_marker.header.frame_id = "map"
            visual_marker.header.stamp = self.get_clock().now().to_msg()
            visual_marker.ns = "coordinates"
            visual_marker.id = len(self.marker_array.markers)
            visual_marker.type = Marker.SPHERE
            visual_marker.action = Marker.ADD

            visual_marker.pose.position = Point(x=lon, y=lat, z=0.0)
            visual_marker.scale.x = 0.5
            visual_marker.scale.y = 0.5
            visual_marker.scale.z = 0.5
            visual_marker.color.r = 0.0
            visual_marker.color.g = 1.0
            visual_marker.color.b = 0.0
            visual_marker.color.a = 1.0

            self.marker_array.markers.append(visual_marker)

        self.marker_publisher.publish(self.marker_array)

    def navigate_route(self):
        self.get_logger().info("Navigating through waypoints...")
        
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        for lat, lon in self.robot_route:
            pose.pose.position.x = lon
            pose.pose.position.y = lat
            route_poses.append(deepcopy(pose))

        self.navigator.waitUntilNav2Active()
        self.navigator.followWaypoints(route_poses)

        i = 0
        while rclpy.ok() and not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Executing current waypoint: %d/%d' % (feedback.current_waypoint + 1, len(route_poses)))

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Route complete!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Route was canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Route failed!')

def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
