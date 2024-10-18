import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import cv2
import yaml
import numpy as np
from firebase_admin import db, credentials
import firebase_admin
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

def parse_coordinates(data):
    """Firebase에서 가져온 문자열 형태의 좌표 데이터를 (위도, 경도) 튜플로 변환."""
    data = data.strip("[]").split(", ")
    coordinates = []

    for item in data: #y,x
        lat_lon = item.split(" ")
        lat = float(lat_lon[0])
        lon = float(lat_lon[1])
        coordinates.append((lat, lon))

    return coordinates

class In(Node):
    def __init__(self):
        super().__init__('in')

        self.latitude = 0.0
        self.longitude = 0.0

        # Firebase 초기화
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/hgy/ict/test2-bc888-firebase-adminsdk-m10sa-2b192d12cd.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://test2-bc888-default-rtdb.firebaseio.com'
            })
        self.ref = db.reference('admin')

        # TF 브로드캐스터 초기화
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

         # map 메시지 발행 설정
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_timer = self.create_timer(1.0, self.publish_map_callback)

        # MarkerArray 메시지 발행 설정
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.marker_timer = self.create_timer(1.0, self.publish_data_from_firebase)

        self.product_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.product_marker_timer = self.create_timer(1.1, self.publish_product_data_from_firebase)

        self.transform_timer = self.create_timer(1.0, self.broadcast_transform)  # 0.1초마다 호출


        self.marker_array = MarkerArray() # db에서 받아온 위치 좌표 마커위함(녹색)
        self.product_marker_array = MarkerArray() #product location

        self.pgm_file_path = '/home/hgy/gy_ws/src/local_pgm/map/4th_floor_crop.pgm'
        self.yaml_file_path = '/home/hgy/gy_ws/src/local_pgm/map/4th_floor_crop.yaml'

        # PGM 및 YAML 파일 불러오기
        self.load_map(self.pgm_file_path, self.yaml_file_path)
        self.publish_map_callback()

    def load_map(self, pgm_file_path, yaml_file_path):
        """PGM 파일과 YAML 파일을 불러와서 맵 정보를 설정합니다."""
        try:
            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            self.image = cv2.imread(pgm_file_path, cv2.IMREAD_UNCHANGED)
            if self.image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return

            self.resolution = yaml_data.get('resolution')
            self.origin = yaml_data.get('origin')
            self.origin[0] = -self.origin[0]
            negate = yaml_data.get('negate')
            occupied_thresh = yaml_data.get('occupied_thresh')
            free_thresh = yaml_data.get('free_thresh')
            self.get_logger().info(f"resolution: {self.resolution}, origin: {self.origin}")

            if negate == 1:
                self.image = cv2.bitwise_not(self.image)

            self.image_shape = self.image.shape
            self.occupancy_data = []
            for pixel in self.image.flatten():
                if pixel >= occupied_thresh:
                    self.occupancy_data.append(0)  # 장애물 없음
                elif pixel <= free_thresh:
                    self.occupancy_data.append(100)  # 장애물 있음
                else:
                    self.occupancy_data.append(-1)  # 알 수 없음

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

    def publish_map_callback(self): # 전체 pgm 맵을 발행
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.height, grid_msg.info.width = self.image_shape
        grid_msg.info.resolution = self.resolution
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.position.z = self.origin[2]
        grid_msg.info.origin.orientation.w = 0.0

        grid_msg.data = self.occupancy_data
        self.map_publisher_.publish(grid_msg)
        self.get_logger().info("map을 성공적으로 발행했습니다.")

    def publish_data_from_firebase(self):
        """Firebase에서 마커 데이터를 가져와 시각화합니다."""
        marker_data = self.ref.child('marker/robot').get()
        if marker_data:
            coordinates = parse_coordinates(marker_data)
            self.update_visualization_markers(coordinates)


    def update_visualization_markers(self, coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.marker_array.markers.clear()

        for i, (self.lat, self.lon) in enumerate(coordinates):
            self.get_logger().info(f"마커 좌표: lat={self.lat}, lon={self.lon}")

            visual_marker = Marker()
            visual_marker.header.frame_id = "map"
            visual_marker.header.stamp = self.get_clock().now().to_msg()
            visual_marker.ns = "coordinates"
            visual_marker.id = i
            visual_marker.type = Marker.SPHERE
            visual_marker.action = Marker.ADD

            visual_marker.pose.position = Point(x=self.lon, y=self.lat, z=0.0)
            visual_marker.scale.x = 0.01
            visual_marker.scale.y = 0.01
            visual_marker.scale.z = 0.01

            visual_marker.color.r = 0.0
            visual_marker.color.g = 1.0
            visual_marker.color.b = 0.0
            visual_marker.color.a = 1.0

            visual_marker.lifetime = Duration(sec=0, nanosec=0)
            self.marker_array.markers.append(visual_marker)

        self.marker_publisher_.publish(self.marker_array)
        self.get_logger().info(f"{len(self.marker_array.markers)}개의 마커가 발행되었습니다.")


    def publish_product_data_from_firebase(self):
        product_marker_data = self.ref.child('marker/destination').get()
        if product_marker_data:
            coordinates = parse_coordinates(product_marker_data)
            self.update_visualization_product_markers(coordinates)


    def update_visualization_product_markers(self, coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.product_marker_array.markers.clear()

        for j, (self.lat, self.lon) in enumerate(coordinates):
            self.get_logger().info(f"마커 좌표: lat={self.lat}, lon={self.lon}")

            product_visual_marker = Marker()
            product_visual_marker.header.frame_id = "map"
            product_visual_marker.header.stamp = self.get_clock().now().to_msg()
            product_visual_marker.ns = "coordinates"
            product_visual_marker.id = j
            product_visual_marker.type = Marker.SPHERE
            product_visual_marker.action = Marker.ADD

            product_visual_marker.pose.position = Point(x=self.lon, y=self.lat, z=0.0)
            product_visual_marker.scale.x = 0.01
            product_visual_marker.scale.y = 0.01
            product_visual_marker.scale.z = 0.01

            product_visual_marker.color.r = 1.0
            product_visual_marker.color.g = 0.0
            product_visual_marker.color.b = 0.0
            product_visual_marker.color.a = 1.0

            product_visual_marker.lifetime = Duration(sec=0, nanosec=0)
            self.product_marker_array.markers.append(product_visual_marker)

        self.marker_publisher_.publish(self.product_marker_array)
        self.get_logger().info(f"{len(self.product_marker_array.markers)}개의 마커가 발행되었습니다.")


    def broadcast_transform(self):
    # map과 base_link 간의 변환도 설정
        t_map_to_base = geometry_msgs.msg.TransformStamped()
        t_map_to_base.header.stamp = self.get_clock().now().to_msg()
        t_map_to_base.header.frame_id = 'map'
        t_map_to_base.child_frame_id = 'base_link'  # base_link로 설정

        t_map_to_base.transform.translation.x = self.origin[0]
        t_map_to_base.transform.translation.y = self.origin[1]
        t_map_to_base.transform.translation.z = 0.0

        t_map_to_base.transform.rotation.x = 0.0
        t_map_to_base.transform.rotation.y = 0.0
        t_map_to_base.transform.rotation.z = 0.0
        t_map_to_base.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t_map_to_base)
        self.get_logger().info("map과 base_link 프레임 변환을 발행했습니다.")


def main(args=None):

    rclpy.init(args=args)
    tf_broadcaster = In()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



