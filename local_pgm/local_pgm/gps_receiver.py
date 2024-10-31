import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time, logging

def parse_gps_data(nmea_sentence):
    parts = nmea_sentence.split(',')

    # 파트의 길이가 10보다 클 경우
    if len(parts) > 10:
        # latitude와 longitude가 숫자인지 확인
        try:
            latitude = float(parts[9])
            longitude = float(parts[10])
            print(nmea_sentence)
            # logging.info(f'\033[92m{nmea_sentence}\033[0m')
            return latitude, longitude
        except ValueError:
            # 변환 실패 시 None 반환
            return None, None
    else:
        return None, None

class GpsReceiver(Node):
    def __init__(self):
        super().__init__('gps_receiver')

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.previous_latitude = None  # 이전 위도 값 저장
        self.previous_longitude = None  # 이전 경도 값 저장
        serial_port = '/dev/ttyUSB0'
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            time.sleep(2)  # 시리얼 포트 안정화 대기
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 오류: {e}")
            self.ser = None  # 초기화 실패 시 None으로 설정

    def timer_callback(self):
        if self.ser is not None and self.ser.in_waiting > 0:  # self.ser이 None이 아닐 때만
            nmea_sentence = self.ser.readline().decode('ascii', errors='replace').strip()
            latitude, longitude = parse_gps_data(nmea_sentence)

            if latitude is not None and longitude is not None:
                # 기존 값과 비교
                if (self.previous_latitude is not None and self.previous_longitude is not None and 
                    (self.previous_latitude != latitude or self.previous_longitude != longitude)):
                    self.get_logger().info(f"\033[94m위도/경도 값이 변경되었습니다: 이전 위도: {self.previous_latitude}, 이전 경도: {self.previous_longitude} -> 새 위도: {latitude}, 새 경도: {longitude}\033[0m]")
                else:
                    # 동일한 값일 때 색상 변경된 로그
                    self.get_logger().info("\033[94m same lat,lon\033[0m")  # 블루 색상

                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps_link'
                msg.latitude = latitude
                msg.longitude = longitude

                self.publisher_.publish(msg)  # 발행
                self.get_logger().info(f"GPS 데이터 발행: 위도: {latitude}, 경도: {longitude}")

                # 이전 값 업데이트
                self.previous_latitude = latitude
                self.previous_longitude = longitude

def main(args=None):
    rclpy.init(args=args)
    gps_receiver = GpsReceiver()
    rclpy.spin(gps_receiver)
    gps_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()