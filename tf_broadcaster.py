import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import cv2
import yaml
import numpy as np
from firebase_admin import db, credentials
import firebase_admin
from pyproj import Proj, transform  # pyproj의 transform 함수 사용
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

latitude = 37.631679
longitude = 127.054740
altitude = 0.0

def parse_coordinates(data):
    data = data.strip("[]").split(", ")
    coordinates = []

    for item in data:
        lat_lon = item.split(" ")
        lat = float(lat_lon[1])
        lon = float(lat_lon[3])
        coordinates.append((lat, lon))

    return coordinates

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        # Firebase 초기화 (한 번만 실행)
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/hgy/ict/test2-bc888-firebase-adminsdk-m10sa-2b192d12cd.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://test2-bc888-default-rtdb.firebaseio.com'
            })
        self.ref = db.reference('admin')

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        

        # UTM-K 좌표계 정의
        self.wgs84 = Proj(init='epsg:4326')  # WGS84 좌표계 (위도, 경도)
        self.utm_k = Proj(proj="utm", zone=52, ellps="WGS84", south=False)  # UTM-K 좌표계

        # /gps/fix 토픽 발행 설정
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)  # 3초마다 GPS 데이터 발행

        # map 메시지 발행 설정
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_publisher_timer = self.create_timer(2.0, self.publish_map_callback)

        # MarkerArray 메시지 발행 설정 (마커 표시)
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.marker_timer = self.create_timer(3.1, self.publish_data_from_firebase)  # 3초마다 발행

        self.marker_array = MarkerArray()

        # PGM 및 YAML 파일 경로 설정
        self.pgm_file_path = ('/home/hgy/gy_ws/src/local_pgm/map/empty_map3_keep.pgm')
        self.yaml_file_path = ('/home/hgy/gy_ws/src/local_pgm/map/empty_map3_keep.yaml')

        # 자른 이미지의 범위 설정 (xmin, ymin, xmax, ymax)
        self.crop_range = (0, 50, 50, 100)


        # PGM 및 YAML 파일 불러오기
        self.load_map(self.pgm_file_path, self.yaml_file_path)

    def load_map(self, pgm_file_path, yaml_file_path):
        # PGM 파일 읽기
        try:
            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            image = cv2.imread(pgm_file_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return
            
            image = cv2.flip(image, 1)  # 1은 좌우 반전

            x_min, y_min, x_max, y_max = self.crop_range
            cropped_image = image[y_min:y_max, x_min:x_max]

            # YAML 데이터에서 맵 정보 추출
            self.resolution = yaml_data.get('resolution')
            self.origin = yaml_data.get('origin')
            negate = yaml_data.get('negate')
            occupied_thresh = yaml_data.get('occupied_thresh')
            free_thresh = yaml_data.get('free_thresh')
            self.get_logger().info(f"resolution: {self.resolution}, origin: {self.origin}" )

             # 이미지 데이터 처리
            if negate == 1:
                cropped_image = cv2.bitwise_not(cropped_image)

            # 픽셀 값을 map 데이터로 매핑
            self.occupancy_data = []
            for pixel in cropped_image.flatten():
                if pixel >= occupied_thresh:
                    self.occupancy_data.append(0)
                elif pixel <= free_thresh:
                    self.occupancy_data.append(100)
                else:
                    self.occupancy_data.append(-1)

            self.image_shape = cropped_image.shape
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
            """
            # 이미지 데이터 처리
            if negate == 1:
                image = cv2.bitwise_not(image)

            # 픽셀 값을 map 데이터로 매핑
            self.occupancy_data = []
            for pixel in image.flatten():
                if pixel >= occupied_thresh:
                 self.occupancy_data.append(0)  # 255는 장애물 없음으로 간주 (0)
                elif pixel <= free_thresh:
                    self.occupancy_data.append(100)    # 0은 장애물 있음으로 간주 (100)
                else:
                    self.occupancy_data.append(-1)   # 미확인 영역
            else:
                self.get_logger().error("헤더가 P2가 아닙니다.")
            self.image_shape = image.shape
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
            """

           
    def publish_map_callback(self):
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

    def display_image(self):
        cv2.waitKey(1)  # 비동기적으로 키 입력을 기다림

    def publish_data_from_firebase(self):
        # Firebase에서 데이터 가져오기
        marker_data = self.ref.child('marker/data').get()

        if marker_data:
            coordinates = parse_coordinates(marker_data)
            self.update_visualization_markers(coordinates)
            
    def update_visualization_markers(self, coordinates):
        self.marker_array.markers.clear()

        for i, (lat, lon) in enumerate(coordinates):
            
            visual_marker = Marker()
            visual_marker.header.frame_id = "map"
            visual_marker.header.stamp = self.get_clock().now().to_msg()
            visual_marker.ns = "coordinates"
            visual_marker.id = i
            visual_marker.type = Marker.SPHERE
            visual_marker.action = Marker.ADD

            visual_marker.pose.position = Point(x=lon, y=lat, z=0.0)
            visual_marker.scale.x = 0.0005
            visual_marker.scale.y = 0.0005
            visual_marker.scale.z = 0.0000
        
            visual_marker.color.r = 0.0
            visual_marker.color.g = 1.0
            visual_marker.color.b = 0.0
            visual_marker.color.a = 1.0

            visual_marker.lifetime = Duration(sec=0, nanosec=0)
            self.marker_array.markers.append(visual_marker)

        self.marker_publisher_.publish(self.marker_array)
        self.get_logger().info("MarkerArray를 발행했습니다.")

    def timer_callback(self):
        latitude = 37.631679
        longitude = 127.054740
        altitude = 0.0

        # GPS 데이터 발행
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude

        self.publisher_.publish(msg)
        self.get_logger().info(f"GPS 데이터 발행: 위도: {msg.latitude}, 경도: {msg.longitude}, 고도: {msg.altitude}")

        self.broadcast_transform(msg.latitude, msg.longitude, msg.altitude)

    def broadcast_transform(self, gps_lat, gps_lon, gps_alt):
        # WGS84 좌표계에서 UTM-K 좌표계로 변환 (transform 사용)
            utm_x, utm_y = transform(self.wgs84, self.utm_k, gps_lon, gps_lat)  # 경도, 위도 순서로 변환
            utm_z = gps_alt

            t_map_to_utm = geometry_msgs.msg.TransformStamped()
            t_map_to_utm.header.stamp = self.get_clock().now().to_msg()
            t_map_to_utm.header.frame_id = 'map'
            t_map_to_utm.child_frame_id = 'utm_k'

            t_map_to_utm.transform.translation.x = float(utm_x)
            t_map_to_utm.transform.translation.y = float(utm_y)
            t_map_to_utm.transform.translation.z = float(utm_z)

            t_map_to_utm.transform.rotation.x = 0.0
            t_map_to_utm.transform.rotation.y = 0.0
            t_map_to_utm.transform.rotation.z = 0.0
            t_map_to_utm.transform.rotation.w = 1.0

            self.broadcaster.sendTransform(t_map_to_utm)
            self.get_logger().info(f"위도, 경도, 고도: {gps_lat}, {gps_lon}, {gps_alt}")
            self.get_logger().info(f"UTM 좌표로 변환: {utm_x}, {utm_y}, {utm_z}")
            self.get_logger().info("map과 utm_k 프레임 변환을 발행했습니다.")

            # map과 gps 프레임 간의 변환도 설정
            t_map_to_gps = geometry_msgs.msg.TransformStamped()
            t_map_to_gps.header.stamp = self.get_clock().now().to_msg()
            t_map_to_gps.header.frame_id = 'map'
            t_map_to_gps.child_frame_id = 'gps_frame'

            t_map_to_gps.transform.translation.x = gps_lon
            t_map_to_gps.transform.translation.y = gps_lat
            t_map_to_gps.transform.translation.z = 0.0

            t_map_to_gps.transform.rotation.x = 0.0
            t_map_to_gps.transform.rotation.y = 0.0
            t_map_to_gps.transform.rotation.z = 0.0
            t_map_to_gps.transform.rotation.w = 1.0

            self.broadcaster.sendTransform(t_map_to_gps)
            self.get_logger().info("map과 gps 프레임 변환을 발행했습니다.")

            # gps -> base_link 변환
            t_gps_to_base = geometry_msgs.msg.TransformStamped()
            t_gps_to_base.header.stamp = self.get_clock().now().to_msg()
            t_gps_to_base.header.frame_id = 'gps_frame'
            t_gps_to_base.child_frame_id = 'base_link'

            t_gps_to_base.transform.translation.x = 0.0
            t_gps_to_base.transform.translation.y = 0.0
            t_gps_to_base.transform.translation.z = 0.0

            t_gps_to_base.transform.rotation.x = 0.0
            t_gps_to_base.transform.rotation.y = 0.0
            t_gps_to_base.transform.rotation.z = 0.0
            t_gps_to_base.transform.rotation.w = 1.0

            self.broadcaster.sendTransform(t_gps_to_base)
            self.get_logger().info("gps와 base_link 프레임 변환을 발행했습니다.")

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TfBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()