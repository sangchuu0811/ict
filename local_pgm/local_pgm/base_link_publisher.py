import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

class BaseLinkPublisher(Node):
    def __init__(self):
        super().__init__('base_link_publisher')

        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/hgy/ict/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
            })

        # tf2 버퍼와 리스너 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) #TF 변환 정보 실시간으로 수신 버퍼에 저장> 필요할 때 쉽게 조회

        # 타이머 생성 (0.1초마다 위치 퍼블리시)
        self.timer = self.create_timer(0.1, self.publish_base_link_position)

    def publish_base_link_position(self):
        try:
            # map 또는 odom 프레임에서 base_link의 변환을 가져옴
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # base_link의 위치 정보 (x, y, z)를 Point 메시지에 저장
            point = Point()
            point.x = trans.transform.translation.x
            point.y = trans.transform.translation.y
            point.z = trans.transform.translation.z

            # 위치 퍼블리시
            # self.publisher_.publish(point)
            self.get_logger().info(f'Published base_link position: x={point.x}, y={point.y}, z={point.z}')

            data_string = f"[{point.y} {point.x}]"

            # Firebase에 데이터 추가
            ref = db.reference('/admin/marker/base_link')  # 데이터를 추가할 경로
            ref.set(data_string)  # 문자열 형태로 저장
            self.get_logger().info("Data added to Firebase!")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform base_link: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()