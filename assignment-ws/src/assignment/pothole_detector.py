##############
# pothole_detector.py
#
# Author: Robert Stevenson
# Date: 06-01-2024
#
# This file contains the code for the Pothole Detector.
# Used OpenCV and ROS to detect the simple pothole world pink coloured potholes that are on
# the simulated environments road surfaces and records the position of each pothole seen by processing and
# transforming the image and depth data received from the cameras and publishing it to the rest of 
# the ROS network as a PoseArray.
#
# This incorporates the the knowledge learned from the sources workshop and applies them with additional
# techniques in order to carry out the task of detecting potholes in the environment.
# This include the decision to filter duplicate pothole locations by using the Euclidean distance between
# the new and existing pothole locations and a defined filter radius.
# Additionally, we restrict whether to record the pothole locations that where detected but are not within
# the defined min and max depth value range.
# This allow for more control of the acceptable noise and how we can reduce it in so that the positions
# of the potholes are more accurate and reliable during operations. 
##############

import rclpy
from rclpy.node import Node
from rclpy import qos

from cv2 import cvtColor, imshow, inRange, resizeWindow, circle, FONT_HERSHEY_SIMPLEX, putText
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, waitKey
from cv2 import resize, INTER_CUBIC
from cv2 import findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, drawContours # for counting the potholes masked
from cv2 import moments # for getting the center of the potholes

from numpy import mean
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

import image_geometry
import math

from cv_bridge import CvBridge, CvBridgeError

class PotholeDetector(Node):
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    # color2depth_aspect_h = (71.0/640) / (67.9/480) # Real Robot Values
    # calculated as (color_vertical_FOV/color_width) / (depth_vertical_FOV/depth_width) from the dabai camera parameters
    # color2depth_aspect_v = (43.7/640) / (45.3/480) # Real Robot Values
    color2depth_aspect_h = 1.0
    color2depth_aspect_v = 1.0

    # detector parameters that can be changes to tune the performance for detection of potholes 
    filter_radius = 0.15 # 0.15 is currently the best value
    min_depth = 0.3
    max_depth = 1.15

    camera_model = None
    image_depth_ros = None

    # storage ofd pothole infomation
    pothole_locations = []
    depth_values = []
    pot_locations_posearray = PoseArray()

    # font which we will be using to display pothole count in the preview window
    font = FONT_HERSHEY_SIMPLEX 

    def __init__(self):
        super().__init__('pothole_detection')
        self.bridge = CvBridge()
        
        # get the camera info
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                        self.camera_info_callback, 
                                        qos_profile=qos.qos_profile_sensor_data)

        # sub to the camera image stream's topic and handle the image data in the approriate callback
        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=qos.qos_profile_sensor_data) # Set QoS Profile
        # sub to the depth image stream's topic and handle the depth data in the approriate callback
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)
        
        # publish the pothole locations as a posearray
        self.pothole_locations_pub = self.create_publisher(PoseArray, 
                                                         '/limo/pothole_locations',
                                                         10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # get the tf transform of one frame to another
    def get_tf_transform(self, target_frame, source_frame):
        """
        Retrieves the transformation from the given target frame to the source frame.

        Parameters:
            target_frame (str): The target frame.
            source_frame (str): The source frame.

        Returns:
            geometry_msgs.msg.TransformStamped or None: The transformation from the target frame to the source frame,
            or None if the transformation lookup fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    
    def camera_info_callback(self, data):
        """
        Initializes the camera model if not already initialized, and updates it with the given camera info data.

        Parameters:
            data (CameraInfo): The camera info data used to update the camera model.
        
        Returns:
            None
        """
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        
    def image_depth_callback(self, data):
        """
        Set the value of self.image_depth_ros to the provided data.

        Parameters:
            data (any): The data to be assigned to self.image_depth_ros.

        Returns:
            None.
        """
        self.image_depth_ros = data

    def image_callback(self, data):
        """
        This function is the callback function for the image topic. It receives the image data and performs the following tasks:
        1. Checks if the camera_model and depth image are available, else returns.
        2. Converts the images from ROS format to OpenCV format.
        3. Converts the color image to the HSV color space and applies a mask to isolate the pink pothole color.
        4. Finds the contours of the masked potholes and the centroid of each pothole.
        5. Filters out potholes based on if the depth values are within our defined min and max depth values.
        6. Converts the pothole coordinates to the camera frame.
        7. Checks if the pothole coordinates are already stored by seeing the Euclidean distance between new 
           and existing pothole coordinates are within the filter radius. 
           If not, adds them to the list of pothole locations.
        8. Draws the pothole centroids and contours on the anotated image thats being displayed as a preview.
        9. Publishes the pothole locations as a PoseArray to the rest of the ROS network.
        10. Displays the annotated image and depth image.
        
        Parameters:
        - data: The image data received from the image topic.
        
        Return:
        - None
        """

        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        image_annotated = image_color
        # convert the color image to the HSV color space
        image_color = cvtColor(image_color, COLOR_BGR2HSV)

        # The clour pink's HSV values for our mask that are scaled between 0-255 
        # and have some "padding" either side of the values to minimize the loss of info
        mask = inRange(image_color, (145, 50, 50), (170, 255, 255))

        # count the number of detected potholes that where masked
        potholes, _ = findContours(mask, RETR_TREE, CHAIN_APPROX_SIMPLE)

        # get the centroids of each object
        pothole_centroids_img = []
        pothole_centroids_depth = []
        for pothole in potholes:
            # calculate the moments
            p_moments = moments(pothole)

            # prevent div by 0
            if p_moments['m00'] == 0:
                print('No object detected.')
                return
            
            # Calc the centroid (referenced: https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/)
            img_x = int(p_moments['m10'] / p_moments['m00'])
            img_y = int(p_moments['m01'] / p_moments['m00'])
            pothole_centroids_img.append((img_x, img_y))

            d_x = image_depth.shape[1]/2 + (img_x - image_color.shape[1]/2)*self.color2depth_aspect_h
            d_y = image_depth.shape[0]/2 + (img_y - image_color.shape[0]/2)*self.color2depth_aspect_v 
            pothole_centroids_depth.append((d_x, d_y))

            # get the depth reading at the centroid location
            depth_value = image_depth[int(d_y), int(d_x)]

            # accept of reject a pothole based on if the depth reading is within a certain range
            if depth_value < self.min_depth or depth_value > self.max_depth:
                continue

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((img_x, img_y)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            # NOTE : This could probably be done better so that there are less CPU cycles and be faster when running on a RPI or Jetson Nano 
            if len(self.pothole_locations) == 0: # if nothing is stored yet, add the location
                # add the location
                print("Pothole location added (nothing is stored yet)")
                # add it to the posearray of the locations 
                pose = Pose()
                pose.orientation.z = 0.707
                pose.orientation.w = 0.707
                pose.position.x = camera_coords[0]
                pose.position.y = camera_coords[1]
                pose.position.z = camera_coords[2]
                # transform the coords into the odom frame
                t = self.get_tf_transform('odom', 'depth_link')
                p_cam = do_transform_pose(pose, t)
                self.pot_locations_posearray.poses.append(p_cam)
                # add the pothole location
                print("(add) Pothole location: ", p_cam.position)
                self.pothole_locations.append(p_cam.position)

                # add the depth value
                self.depth_values.append(depth_value)            
            else: # else check if the location is already stored
                add = True

                # convert to a pose
                pose = Pose()
                pose.orientation.z = 0.707
                pose.orientation.w = 0.707
                pose.position.x = camera_coords[0]
                pose.position.y = camera_coords[1]
                pose.position.z = camera_coords[2]
                # transform the coords into the odom frame
                t = self.get_tf_transform('odom', 'depth_link')
                p_cam = do_transform_pose(pose, t)

                # check if the location is already stored
                for location in self.pothole_locations:
                    # print(f"(Check) location: [{location.x}, {location.y}, {location.z}] (Check) p_cam: {p_cam.position}")

                    # Check if the pothole coords are close to already stored points, if so, don't add them, else add them
                    # do this via calculating the euclidian distance between the two points
                    dist = math.sqrt((location.x - p_cam.position.x)**2 +
                                    (location.y - p_cam.position.y)**2 +
                                    (location.z - p_cam.position.z)**2)
                    print("Euclidean Distance: ", dist)
                    if dist < self.filter_radius: # if the distance is less than 0.11m, don't add the location
                        # don't add the location
                        print("Pothole location already stored")
                        add = False
                        break

                # point hasn't been added
                if add:    
                    # add the location
                    print("Pothole location added (as not already stored)")
                    # add it to the posearray of the locations 
                    self.pot_locations_posearray.poses.append(p_cam)
                    # add the pothole location
                    print("(add) Pothole location: ", p_cam.position)
                    self.pothole_locations.append(p_cam.position)
                    # add the depth value
                    self.depth_values.append(depth_value)

            # draw the centroids on the depth and annotated image for potholes that 
            # are in range and would be processed for storing
            circle(image_annotated, (img_x, img_y), 2, (0, 255, 0), -1)
            circle(image_depth, (int(d_x), int(d_y)), 2, (255, 255, 255), -1)

        # publish the pothole locations
        self.pot_locations_posearray.header.frame_id = 'odom'
        self.pothole_locations_pub.publish(self.pot_locations_posearray)
        print(f"published {len(self.pot_locations_posearray.poses)} potholes")
        # draw the pothole's contours on the annotated image (even if they are not stored)
        for pothole in potholes:
            drawContours(image_annotated, [pothole], -1, (255, 0, 0), 1)
        
        
        # resize the windows
        image_depth *= 1.0/5.0 # scale for visualisation (max range 5.0 m)

        # draw counter of potholes to image deteion window
        image_annotated = putText(image_annotated, f"Detected potholes: {len(self.pothole_locations)}", (10, 30), FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        image_annotated = resize(image_annotated, (0,0), fx=0.5, fy=0.5) # rescale for visualisation
        imshow("Detection Image Window", image_annotated) # show the annotated image
        image_depth = resize(image_depth, (0,0), fx=0.5, fy=0.5) # rescale for visualisation
        imshow("Depth Image Window", image_depth) # show the depth image
        resizeWindow('Detection Image Window', 600,600)
        resizeWindow('Depth Image Window', 600,600)
        
        # print the pothole locations and count to the terminal
        print(f"Detected potholes: {len(self.pothole_locations)}")
        print(f"pothole locations: {self.pothole_locations}")
        # print(f"depth values: {self.depth_values}")
        # print(f"pothole centroids: {pothole_centroids_img}")
        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    # start the detector node
    rclpy.spin(pothole_detector)
    pothole_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()