import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        # 创建一个发布者，发布到 /camera/image_raw 话题
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        # 定时器，大约以 30fps 的帧率发布视频
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # ⚠️ 请将这里的路径替换为你 Ubuntu 系统下的视频真实路径！
        self.video_path = '/home/benlmy/rt_vision-1/final_project/test.mp4' 
        self.cap = cv2.VideoCapture(self.video_path)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 将 OpenCV 格式转换为 ROS2 消息格式并发布
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            # 视频播完后自动循环
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    print("✅ 视频发布节点已启动，正在向 /camera/image_raw 广播图像流...")
    rclpy.spin(video_publisher)
    video_publisher.cap.release()
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()