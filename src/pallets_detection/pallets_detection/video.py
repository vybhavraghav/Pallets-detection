import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
os.path

class VideoPub(Node):
    def __init__(self):
        super().__init__('video')

        topic_name = '/camera/image_raw'
        pwd = os.path.dirname(os.path.abspath(__file__))
        
        print(pwd)
        self.vid_file = os.path.join(pwd, 'Rinkit! Warehouse Walkthrough Tour.mp4')
        self.cap = cv2.VideoCapture(self.vid_file)
        self.br = CvBridge()
        self.vid_pub = self.create_publisher(Image, topic_name, 10)
        self.create_timer(0.05, self.play_video)


    def play_video(self):
        ret, frame = self.cap.read()  # Read a single frame
        # img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if not ret:
            print("End of video or error reading frame.")
            self.cap = cv2.VideoCapture(self.vid_file)
        else:
            # Display the frame in a window
            self.vid_pub.publish(self.br.cv2_to_imgmsg(frame))
            cv2.imshow('Video Playback', frame)
            cv2.waitKey(1)


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