import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
os.path

class VideoPub(Node):
    def __init__(self):
        super().__init__('camera')

        topic_name = '/robot1/zed2i/left/image_rect_color'
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.vid_pub = self.create_publisher(Image, topic_name, 10)
        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        
        ret, frame = self.cap.read()     
        print(ret)
        if ret == True:
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.vid_pub.publish(self.br.cv2_to_imgmsg(img))
            cv2.imshow('video', frame)
            cv2.waitKey(1)
        self.get_logger().info('Publishing video frame')




def main(args=None):
    rclpy.init(args=args)
    node = VideoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Destroy OpenCV windows
        cv2.destroyAllWindows()

  
if __name__ == '__main__':
  main()