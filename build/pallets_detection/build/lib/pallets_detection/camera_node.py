import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class VideoPub(Node):
    def __init__(self):
        super().__init__('camera')

        topic_name = '/camera/image_raw'
        self.cap = cv2.VideoCapture()
        self.br = CvBridge()
        self.vid_pub = self.create_publisher(Image, topic_name, 10)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        ret, frame = self.cap.read()     
        if ret == True:
            self.vid_pub.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    simple_pub_sub = VideoPub()
    rclpy.spin(simple_pub_sub)
    simple_pub_sub.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()