#! /usr/bin/env python3


import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
from ultralytics import YOLO
import os
import numpy as np


    
class PalletDetection(Node):
    def __init__(self):
        super().__init__('detection')

        topic_name= '/robot1/zed2i/left/image_rect_color'

        self.publisher_ = self.create_publisher(Image, 'segmented_image' , 10)

        self.br = CvBridge()
        
        current_directory = os.path.dirname(__file__)
        
        model_path =os.path.abspath(os.path.join(current_directory, "..",'..','..', 'src','runs/segment/train/weights/best.pt'))
        print(model_path)
        self.detection_model = YOLO(model_path)

        self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)
        self.subscription 
        self.br = CvBridge()


    # def timer_callback(self):
    #     ret, frame = self.cap.read()     
    #     if ret == True:
    #         self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
    #     self.get_logger().info('Publishing video frame')


    def img_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        # current_frame = cv2.resize(current_frame, (640,640))
        segmented_image = self.detect_and_mask(current_frame)
        detection_img = self.br.cv2_to_imgmsg(segmented_image)


        self.publisher_.publish(msg = detection_img)
        self.get_logger().info('Publishing video frame')

    def detect_and_mask(self, image):

        # Run inference
        results = self.detection_model(image)

        # Get the first result (for simplicity)
        result = results[0]

        # Overlay segmentation masks on the image
        segmented_image = image.copy()
        try:
            for mask, box, cls, conf in zip(result.masks.xy, result.boxes.xyxy, result.boxes.cls, result.boxes.conf):
                # Convert mask coordinates to a binary mask (size matches input image)
                if conf>0.3:
                    mask_points = np.array(mask, dtype=np.int32)
                    if cls == 1:
                        color = (0, 0, 255)  # Green for the mask overlay
                    else:
                        color = (0, 255, 0)  # red for the mask overlay
                    
                    cv2.fillPoly(segmented_image, [mask_points], color)

                    segmented_image = cv2.addWeighted(image, 0.5, segmented_image, 0.5, 0)

                    # Draw bounding boxes and labels
                    x1, y1, x2, y2 = map(int, box)
                    label = f"{self.detection_model.names[int(cls)]} {conf:.2f}"
                    cv2.rectangle(segmented_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue bounding box
                    cv2.putText(segmented_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        except:
            self.get_logger().info("None detected")
        # Optionally display or save the image
        cv2.imshow("Segmented Image", segmented_image)
        cv2.waitKey(1)

        return segmented_image



def main(args=None):
    rclpy.init(args=args)
    node = PalletDetection()

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