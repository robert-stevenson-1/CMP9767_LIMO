import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageConverter(Node):

    def __init__(self):
        super().__init__('opencv_test')
        self.bridge = CvBridge()
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=self.qos_profile) # Set QoS Profile
        
    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("masked")
        namedWindow("canny")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.5, fy=0.5, interpolation = INTER_CUBIC)

        # mask = inRange(cv_image, (0, 150, 150), (255, 255, 255))
        # imshow("masked", mask)
        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)

        # mask out the orange of the traffic Cone
        lower_orange = (0, 150, 150)
        upper_orange = (255, 255, 255)
        mask = inRange(cv_image, lower_orange, upper_orange)
        # show the mask
        imshow("masked", mask)

        img3 = Canny(hsv_img, 10, 200)
        imshow("canny", img3)

        imshow("Image window", cv_image)
        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()