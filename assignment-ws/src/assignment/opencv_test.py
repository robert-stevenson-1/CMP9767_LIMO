import rclpy
from rclpy.node import Node
from rclpy import qos
from cv2 import namedWindow, cvtColor, imshow, inRange, resizeWindow, circle

from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, waitKey
from cv2 import blur, Canny, resize, INTER_CUBIC

from cv2 import findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, drawContours # for counting the potholes masked
from cv2 import contourArea # for getting the size of the potholes 
from cv2 import moments # for getting the center of the potholes

from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageConverter(Node):

    def __init__(self):
        super().__init__('opencv_test')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=qos.qos_profile_sensor_data) # Set QoS Profile
        
    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("masked")
        namedWindow("canny")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.5, fy=0.5, interpolation = INTER_CUBIC)
        cv_image_original = cv_image
        cv_image = cvtColor(cv_image, COLOR_BGR2HSV)

        # max RGB value: 229,74,200
        # min RGB value: 213,0,197
        # mask = inRange(cv_image, (205, 0, 190), (235, 80, 255))
        # max HSV value: 321,48,77
        # min HSV value: 304,100,84
        mask = inRange(cv_image, (145, 50, 50), (170, 255, 255))
        imshow("masked", mask)
        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        img3 = Canny(gray_img, 10, 200)
        imshow("canny", img3)

        imshow("Image window", cv_image)

        # count the number of detected potholes that where masked
        potholes, _ = findContours(mask, RETR_TREE, CHAIN_APPROX_SIMPLE)

        # get the size of each pothole and the centroids of each object
        pothole_sizes = []
        pothole_centroids = []
        for pothole in potholes:
            pothole_sizes.append(contourArea(pothole))
            # calculate the moments
            p_moments = moments(pothole)

            # prevent div by 0
            if p_moments['m00'] != 0:
                # Calc the centroid (referenced: https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/)
                x = int(p_moments['m10'] / p_moments['m00'])
                y = int(p_moments['m01'] / p_moments['m00'])
                pothole_centroids.append((x, y))
                # draw the centroids on the original image
                circle(cv_image_original, (x, y), 2, (0, 255, 0), -1)


        # draw the potholes on the original image with the contours
        for pothole in potholes:
            drawContours(cv_image_original, [pothole], -1, (255, 0, 0), 1)
        # for pothole in potholes:
        #     # calculate the moments
        #     p_moments = moments(pothole)
        #     # prevent div by 0
        #     if p_moments['m00'] != 0:
        #         # Calc the centroid (referenced: https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/)
        #         x = int(p_moments['m10'] / p_moments['m00'])
        #         y = int(p_moments['m01'] / p_moments['m00'])
        #     circle(cv_image_original, (x, y), 2, (0, 255, 0), -1)
        
        imshow("Original Image window", cv_image_original)
        
        # resize the windows
        resizeWindow('Original Image window', 600,600)
        resizeWindow('Image window', 600,600)
        resizeWindow('masked', 600,600)
        resizeWindow('canny', 600,600)
        
        print(f"Detected potholes: {len(potholes)}")
        print(f"pothole sizes: {pothole_sizes}")
        print(f"pothole centroids: {pothole_centroids}")
        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()