#!/usr/bin/env python3
# encoding: utf-8
import cv2
import time
import math
import rclpy
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class HandViewer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.image = None
        self.frames = 0
        self.last_time = time.time()
        self._fps_text = "FPS: --"
        self._dst_text = "DST: --"

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mpDraw = mp.solutions.drawing_utils

        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')

    def timer_callback(self):
        if self.image is None:
            return

        frame = cv2.flip(self.image, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(frame, hand_landmarks, self.mpHands.HAND_CONNECTIONS)
                h, w, _ = frame.shape
                p4 = hand_landmarks.landmark[4]
                p8 = hand_landmarks.landmark[8]
                x4, y4 = int(p4.x * w), int(p4.y * h)
                x8, y8 = int(p8.x * w), int(p8.y * h)
                cv2.line(frame, (x4, y4), (x8, y8), (0, 255, 255), 2)
                cv2.circle(frame, (x4, y4), 5, (0, 255, 255), -1)
                cv2.circle(frame, (x8, y8), 5, (0, 255, 255), -1)
                dst = math.hypot(x8 - x4, y8 - y4)
                self._dst_text = f'DST: {dst:.2f}'

        self.frames += 1
        dt = max(1e-6, time.time() - self.last_time)
        if dt >= 0.5:
            fps_val = self.frames / dt
            self.frames = 0
            self.last_time = time.time()
            self._fps_text = f'FPS: {fps_val:.1f}'

        cv2.putText(frame, self._fps_text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        cv2.putText(frame, self._dst_text, (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)

        cv2.imshow('hand_viewer', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandViewer('hand_viewer')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
