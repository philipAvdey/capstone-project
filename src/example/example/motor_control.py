from interfaces.msg import ObjectsInfo
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ObjectsInfo, '/yolov11_detect_demo/object_detect', self.detection_callback, 10)

        self.speed = 0.9
        self.turn_speed = 0.7

    def detection_callback(self, msg):
        move_cmd = Twist()
        detected_labels = [obj.class_name for obj in msg.objects]

        for label in detected_labels:
            if 'green' in label:
                move_cmd.linear.x = self.speed
            elif 'red' in label:
                move_cmd.linear.x = -self.speed
            elif 'right' in label:
                move_cmd.angular.z = -self.turn_speed
            elif 'left' in label:
                move_cmd.angular.z = self.turn_speed
            else:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0

        self.publisher.publish(move_cmd)
        self.get_logger().info(f"Detected: {detected_labels}, Moving: {move_cmd.linear.x}, {move_cmd.angular.z}")

def main():
    rclpy.init()
    node = MotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

