import rclpy
from rclpy.node import Node
from rclpy import qos

from cv2 import namedWindow, cvtColor, imshow, inRange, resizeWindow, circle, FONT_HERSHEY_SIMPLEX, putText
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, waitKey
from cv2 import blur, Canny, resize, INTER_CUBIC
from cv2 import findContours, RETR_TREE, CHAIN_APPROX_SIMPLE, drawContours # for counting the potholes masked
from cv2 import contourArea # for getting the size of the potholes 
from cv2 import moments # for getting the center of the potholes

from numpy import mean
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

import image_geometry
import math

from cv_bridge import CvBridge, CvBridgeError

class PotholeDetector(Node):
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    # color2depth_aspect_h = (71.0/640) / (67.9/480)
    # color2depth_aspect_v = (43.7/640) / (45.3/480)
    color2depth_aspect_h = 1.0
    color2depth_aspect_v = 1.0

    filter_radius = 0.15 # 0.15 is currently the best value
    min_depth = 0.3
    max_depth = 1.15

    camera_model = None
    image_depth_ros = None

    pothole_locations = []
    depth_values = []
    pot_locations_posearray = PoseArray()

    # font which we will be using to display pothole count 
    font = FONT_HERSHEY_SIMPLEX 

    def __init__(self):
        super().__init__('pothole_detection')
        self.bridge = CvBridge()
        
        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info',
                                        self.camera_info_callback, 
                                        qos_profile=qos.qos_profile_sensor_data)

        self.image_sub = self.create_subscription(Image, 
                                                    "/limo/depth_camera_link/image_raw",
                                                    self.image_callback,
                                                    qos_profile=qos.qos_profile_sensor_data) # Set QoS Profile
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', 
                                                  self.image_depth_callback,
                                                  qos_profile=qos.qos_profile_sensor_data)
        
        self.pothole_locations_pub = self.create_publisher(PoseArray, 
                                                         '/limo/pothole_locations',
                                                         10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def add_pothole_location(self, location):
        # add it to the posearray of the locations 
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = location[0]
        pose.position.y = location[1]
        pose.position.z = location[2]
        self.pot_locations_posearray.poses.append(pose)
        # transform the coords into the odom frame
        t = self.get_tf_transform('odom', 'depth_link')
        p_cam = do_transform_pose(pose, t)
        # add the pothole location
        print("(add) Pothole location: ", p_cam.position)
        self.pothole_locations.append(p_cam.position)

    def image_callback(self, data):

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

        # crop the image to the middle 3rd of the height
        # image_color = image_color[160: 440, 0: 640] 
        # image_depth = image_depth[160: 440, 0: 640]

        image_annotated = image_color
        image_color = cvtColor(image_color, COLOR_BGR2HSV)


        # The clour pink's HSV values for our mask that are scaled between 0-255 
        # and have some "padding" either side of the values to minimize the loss of info
        mask = inRange(image_color, (145, 50, 50), (170, 255, 255))

        # count the number of detected potholes that where masked
        potholes, _ = findContours(mask, RETR_TREE, CHAIN_APPROX_SIMPLE)

        # get the size of each pothole and the centroids of each object
        # pothole_sizes = []
        pothole_centroids_img = []
        pothole_centroids_depth = []
        for pothole in potholes:
            # pothole_sizes.append(contourArea(pothole))
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
                # remove the pothole
                # potholes.remove(pothole)
                continue

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((img_x, img_y)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

            # NOTE : This could probably be done better 
            if len(self.pothole_locations) == 0: # if nothing is stored yet, add the location
                # add the location
                print("Pothole location added (nothing is stored yet)")
                # add it to the posearray of the locations 
                # self.add_pothole_location(camera_coords)
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
                    # if (abs(location.x - camera_coords[0]) < 0.5 a
                        # don't add the location
                        print("Pothole location already stored")
                        add = False
                        break
                # point hasn't been added
                if add:    
                    # add the location
                    print("Pothole location added (as not already stored)")
                    # self.add_pothole_location(camera_coords)

                    # add it to the posearray of the locations 
                    self.pot_locations_posearray.poses.append(p_cam)
                    # add the pothole location
                    print("(add) Pothole location: ", p_cam.position)
                    self.pothole_locations.append(p_cam.position)

                    # add the depth value
                    self.depth_values.append(depth_value)


            # draw the centroids on the original image
            circle(image_annotated, (img_x, img_y), 2, (0, 255, 0), -1)
            circle(image_depth, (int(d_x), int(d_y)), 2, (255, 255, 255), -1)

        # publish the pothole locations
        self.pot_locations_posearray.header.frame_id = 'odom'
        self.pothole_locations_pub.publish(self.pot_locations_posearray)
        print(f"published {len(self.pot_locations_posearray.poses)} potholes")
        # draw the pothole's contours on the original image 
        for pothole in potholes:
            drawContours(image_annotated, [pothole], -1, (255, 0, 0), 1)
        
        
        # resize the windows
        image_depth *= 1.0/5.0 # scale for visualisation (max range 10.0 m)

        # draw counter of potholes to image deteion window
        image_annotated = putText(image_annotated, f"Detected potholes: {len(self.pothole_locations)}", (10, 30), FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        image_annotated = resize(image_annotated, (0,0), fx=0.5, fy=0.5)
        imshow("Detection Image Window", image_annotated)
        image_depth = resize(image_depth, (0,0), fx=0.5, fy=0.5)
        imshow("Depth Image Window", image_depth)
        resizeWindow('Detection Image Window', 600,600)
        resizeWindow('Depth Image Window', 600,600)
        
        print(f"Detected potholes: {len(self.pothole_locations)}")
        print(f"pothole locations: {self.pothole_locations}")
        # print(f"depth values: {self.depth_values}")
        # print(f"pothole centroids: {pothole_centroids_img}")
        waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    rclpy.spin(pothole_detector)

    pothole_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()