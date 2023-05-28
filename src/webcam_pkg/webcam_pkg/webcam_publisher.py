import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_raw = self.create_publisher(Image, 'webcam/image_raw', 10)
        self.publisher_processed = self.create_publisher(Image, 'webcam/image_processed', 10)
        self.publisher_coordinates = self.create_publisher(Point, 'webcam/webcam_coordinates', 10)
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def publish_images(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                break


            # Publish the raw image
            self.publisher_raw.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Perform face detection
            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # Draw rectangles around the detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), thickness=2)

                # Calculate relative coordinates
                height, width, _ = frame.shape
                center_x = (x + w/2) / width
                center_y = (y + h/2) / height
                

                center_x_draw = int(x + (w/2))
                center_y_draw = int(y + (h/2))

                cv2.circle(frame, (center_x_draw, center_y_draw), radius=10, color=(0, 0, 255), thickness=3)

                # Create and publish Point message for Webcam Coordinates
                coordinates_msg = Point()
                coordinates_msg.x = (center_x - 0.5) * 2  # Normalize to range [-1, 1]
                coordinates_msg.y = (0.5 - center_y) * 2  # Normalize to range [-1, 1]
                coordinates_msg.z = 0.0  # Assuming 2D coordinates

                self.publisher_coordinates.publish(coordinates_msg)

            

            # Publish the processed image
            self.publisher_processed.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            cv2.waitKey(1)

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisherNode()
    webcam_publisher.publish_images()
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
