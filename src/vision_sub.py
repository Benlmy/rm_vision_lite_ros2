import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # 我们用一个简单的点来发布坐标结果
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# 将我们之前调优好的传统视觉代码原封不动放进来
def process_traditional(frame):
    trad_frame = frame.copy()
    b, g, r = cv2.split(frame)
    sub_blue = cv2.subtract(b, r)
    sub_red = cv2.subtract(r, b)
    _, mask_blue = cv2.threshold(sub_blue, 80, 255, cv2.THRESH_BINARY)
    _, mask_red = cv2.threshold(sub_red, 80, 255, cv2.THRESH_BINARY)
    mask = cv2.bitwise_or(mask_blue, mask_red)
    kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_vertical)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    light_bars = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 15 or area > 1500: continue
        x, y, w, h = cv2.boundingRect(cnt)
        if h / float(w) > 1.2:
            light_bars.append((x, y, w, h))

    light_bars.sort(key=lambda b: b[0])
    
    for i in range(len(light_bars)):
        for j in range(i + 1, len(light_bars)):
            x1, y1, w1, h1 = light_bars[i]
            x2, y2, w2, h2 = light_bars[j]
            if abs(h1 - h2) / max(h1, h2) > 0.4: continue
            center_y1, center_y2 = y1 + h1/2, y2 + h2/2
            mean_h = (h1 + h2) / 2
            if abs(center_y1 - center_y2) > mean_h * 0.3: continue
            dist_x = abs(x1 - x2)
            if 1.2 * mean_h < dist_x < 3.5 * mean_h:
                armor_x, armor_y = min(x1, x2), min(y1, y2)
                armor_w = max(x1+w1, x2+w2) - armor_x
                armor_h = max(y1+h1, y2+h2) - armor_y
                if armor_w > armor_h * 1.2:
                    cv2.rectangle(trad_frame, (armor_x, armor_y), (armor_x+armor_w, armor_y+armor_h), (255, 0, 0), 2)
    return trad_frame

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber_node')
        # 1. 订阅图像流
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        # 2. 创建结果发布者：满足大任务“向外发布Topic”的要求
        self.result_pub = self.create_publisher(Point, '/armor_target_center', 10)
        
        self.bridge = CvBridge()
        
        # ⚠️ 请替换为 Ubuntu 下 best.onnx 的真实路径！
        self.model = YOLO('/home/benlmy/rm_vision_lite_ros2/models/best.onnx', task="detect") 

    def listener_callback(self, data):
        # 还原为 OpenCV 图像
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        frame = cv2.resize(frame, (640, 480))

        # 三路处理
        orig_frame = frame.copy()
        cv2.putText(orig_frame, "1. Original", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        trad_frame = process_traditional(frame)
        cv2.putText(trad_frame, "2. Traditional", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        results = self.model.predict(source=frame, conf=0.3, verbose=False)
        yolo_frame = results[0].plot()
        cv2.putText(yolo_frame, "3. YOLO Net", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # 满足大任务要求：通过 Topic 统一发布结果 (提取 YOLO 检测到的第一个装甲板坐标)
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_msg = Point()
            center_msg.x = (x1 + x2) / 2
            center_msg.y = (y1 + y2) / 2
            center_msg.z = 0.0 # 单目没有深度，Z设为0
            self.result_pub.publish(center_msg)

        # 拼接并显示
        combined = cv2.hconcat([orig_frame, trad_frame, yolo_frame])
        cv2.imshow("RoboMaster Final System - ROS2 Node", combined)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    vision_subscriber = VisionSubscriber()
    print("✅ 视觉处理中枢已启动，正在监听画面并发布目标坐标...")
    rclpy.spin(vision_subscriber)
    vision_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()