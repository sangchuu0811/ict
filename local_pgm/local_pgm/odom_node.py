import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # cmd_vel 데이터를 구독
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # 주기적으로 Odometry를 발행하기 위한 타이머 설정
        self.timer = self.create_timer(0.01, self.publish_odometry)

        # 마지막 cmd_vel 데이터를 추적
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

    def cmd_vel_callback(self, msg):
        # cmd_vel 메시지를 받아서 로봇의 위치와 방향 업데이트
        delta_time = 0.1  # 샘플링 시간(초)
        self.current_x += msg.linear.x * math.cos(self.current_theta) * delta_time
        self.current_y += msg.linear.x * math.sin(self.current_theta) * delta_time
        self.current_theta += msg.angular.z * delta_time   

        # 마지막 속도값 갱신
        self.last_linear_x = msg.linear.x
        self.last_angular_z = msg.angular.z

    def publish_odometry(self):
        # Odometry 메시지 발행
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # 로봇의 위치
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0

        # 로봇의 방향 (Quaternion)
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.current_theta)

        # 마지막으로 받은 속도값 사용 (cmd_vel 없으면 0 유지)
        odom.twist.twist.linear.x = self.last_linear_x
        odom.twist.twist.angular.z = self.last_angular_z

        # Odometry 메시지 발행
        self.odom_publisher.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Roll, Pitch, Yaw를 Quaternion으로 변환
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
