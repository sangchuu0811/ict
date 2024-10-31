import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

class FirebasePublisher(Node):

    def __init__(self):
        super().__init__('firebase_publisher')

        # if not firebase_admin._apps:
        #     cred = credentials.Certificate("/home/hgy/ict/test2-bc888-firebase-adminsdk-m10sa-2b192d12cd.json")
        #     firebase_admin.initialize_app(cred, {
        #         'databaseURL': 'https://test2-bc888-default-rtdb.firebaseio.com'
        #     })


        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/hgy/ict/sensemart-8c5c7-firebase-adminsdk-npdg8-fccf1988f9.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://sensemart-8c5c7-default-rtdb.firebaseio.com'
            })
        self.subscription = self.create_subscription(
            Point,
            '/base_link_position',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Point 메시지에서 x와 y 값을 가져옴
        x = msg.x
        y = msg.y
        self.get_logger().info(f'Received: x={x}, y={y}')

        # 데이터를 한 문자열로 결합
        data_string = f"[{y} {x}]"

        # Firebase에 데이터 추가
        ref = db.reference('/admin/marker/base_link')  # 데이터를 추가할 경로
        ref.set(data_string)  # 문자열 형태로 저장
        self.get_logger().info("Data added to Firebase!")

def main(args=None):
    rclpy.init(args=args)
    firebase_publisher = FirebasePublisher()
    rclpy.spin(firebase_publisher)
    firebase_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
