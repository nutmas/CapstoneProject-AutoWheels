#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import yaml

# use for search nearest waypoint
from scipy.spatial import KDTree
import numpy as np

# to process image in classifier in seperate thread
import threading

# variable to set threshold for number of detections before certain for traffic lights
STATE_COUNT_THRESHOLD = 1

# SLOW MODEL = 1
# FAST MODEL = 3

#-----------------------------------------------------------------------------------------------------

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.camera_image = None
        self.lights = []

        self.base_waypoints = None
        self.waypoint_tree = None
        self.waypoints_2d = None

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # initialise flag as classifier is not running
        self.classifier_running = False

        # variable to hold states return from image classifier
        # 1 = Green, 2 = Red, 3 = Yellow, 4 = Unknown
        self.classified_light_state = 4

        # wait until subscribers are ready
        rospy.wait_for_message('/current_pose', PoseStamped)
        rospy.wait_for_message('/base_waypoints', Lane)

        # subscribe to get current localised position of vehicle
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        # subscriber to get base waypoints of track
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # get position of all traffic lights in map world
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        # get image to detect and classify traffic light states
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # create a publish topic for traffic lights
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        # flag to indicate classifier is running
        self.classifier_ready = rospy.Publisher('/classifier_status', Bool, queue_size=1)


        # create a terminate thread
        self.image_ready = threading.Event()
        # create a lock for thread
        self.thread_image_lock = threading.Lock()
        # create a thread that will run classification
        inception_thread = threading.Thread(target=self.get_classification)
        # start thread
        inception_thread.start()

        # initiate main program loop
        self.run_main()

        rospy.spin()

    # -----------------------------------------------------------------------------------------------------

    def get_classification(self):

        while not rospy.is_shutdown():

            # check to see if image is available for processing
            if not self.image_ready.wait(1):
                continue

            # lock the image for retrieval
            self.thread_image_lock.acquire()
            # get image
            image = self.camera_image
            # release lock
            self.thread_image_lock.release()

            # prepare image
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            image_width = self.camera_image.width
            image_height = self.camera_image.height
            # load image into numpy arrray
            np_image = np.array(cv_image).reshape(image_height, image_width, 3).astype(np.uint8)

            # update latest light_state by get classification from net model
            self.classified_light_state =  self.light_classifier.get_classification(np_image)

            # set flag to say classifer is running
            self.classifier_running = True

            # clear flag for image processing
            self.image_ready.clear()

        # if classifer is not running set flag
        self.classifier_running = False

    # -----------------------------------------------------------------------------------------------------

    def run_main(self):
      # set loop to 10Hz - traffic light positions can be updated faster tan images
      rate = rospy.Rate(10)
      while not rospy.is_shutdown():

          # wait until all states are ready
          if self.pose and self.base_waypoints and self.waypoint_tree:

              # publish light message when not using classifier
              light_wp, state = self.process_traffic_lights()

              self.classifier_ready.publish(self.classifier_running)

              # if new state is different to previous state - light has changed
              if self.state != state:
                  # reset state counter
                  self.state_count = 0
                  # set state
                  self.state = state
              # if witnessed 3 same state - steady condition so go confidence
              elif self.state_count >= STATE_COUNT_THRESHOLD:
                  # update traffic light state for action
                  self.last_state = self.state
                  # set light waypint as stop line if light state is RED or YELLOW
                  light_wp = light_wp if state == TrafficLight.RED else -1
                  self.last_wp = light_wp
                  # publish message with latest traffic light state
                  self.upcoming_red_light_pub.publish(Int32(light_wp))
              # if state is same and threshol is not yet reached - keep publishing previous light state
              else:
                  self.upcoming_red_light_pub.publish(Int32(self.last_wp))
              # increment state counter as same state witnessed again
              self.state_count += 1

          rate.sleep()

    # -----------------------------------------------------------------------------------------------------

    # callback for current_pose msg
    def pose_cb(self, msg):

        self.pose = msg

    # -----------------------------------------------------------------------------------------------------

    # callback for base_waypoints msg - load base map to memory
    def waypoints_cb(self, waypoints):
        # store waypoints in the object for continued use
        # latched subscriber - base waypoints never change so should only be stored once
        self.base_waypoints = waypoints

        # if waypoints_2d is empty, then populate with received track points
        if not self.waypoints_2d:
            # convert waypoints to 2D coordinates for each waypoint using list comprehension
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            # populate the waypoint tree variable with the points passed from
            self.waypoint_tree = KDTree(np.array(self.waypoints_2d))

    # -----------------------------------------------------------------------------------------------------

    # callback to store traffic light positions
    def traffic_cb(self, msg):

        # get details from simulator about stop lines positions - for uing simulator state info
        self.lights = msg.lights

    # -----------------------------------------------------------------------------------------------------

    # Callback which runs classifier if traffic light position is in range of vehicle
    def image_cb(self, msg):

        # lock the image variable to update with image received
        self.thread_image_lock.acquire()
        # pass image from msg into variable
        self.camera_image = msg
        # release lock
        self.thread_image_lock.release()
        # set flag to say image is ready
        self.image_ready.set()

    # -----------------------------------------------------------------------------------------------------

    # function to find closest waypoint index for traffic light
    def get_closest_waypoint(self, x, y):

        # load passed in co-ordinates into numpy array
        target_coordinate = np.array([x, y])
        # find closest waypoint index from waypoint tree
        closest_idx = self.waypoint_tree.query(target_coordinate, 1)[1]
        # return waypoint index
        return closest_idx

    # -----------------------------------------------------------------------------------------------------

    def process_traffic_lights(self):
        # Finds closest visible traffic light, if one exists, and determines its location and colour

        # initialise closest light to None
        closest_light = None
        # initialise line waypoint index to None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # get stop line positions from config file
        stop_line_positions = self.config['stop_line_positions']
        # if pose of ego is known
        if(self.pose and self.base_waypoints and self.waypoint_tree):
            # find nearest waypoint index from current car_position
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)


            # find the closest visible traffic light (if one exists)
            # iterate through the whole list of traffic lights
            diff = len(self.base_waypoints.waypoints)
            # update waypoint information in lights
            for i, light in enumerate(self.lights):
                # get stop line waypoint index corresponding to this light loop
                line = stop_line_positions[i]
                # get closest waypoint  feeding in x and y value of stop line
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # find closest stop line waypoint index - store number of indicies between car position and ligh position
                d = temp_wp_idx - car_wp_idx

                # if number of indicies is >0 - so in front of car and not exceeding number of points to search
                if d >= 0 and d < diff:
                    # set closest light, stop line
                    # metres to stop line - greater than 10 looks like car is passed
                    diff = d
                    # xyz positon coordinates of closest light and zw orientation
                    closest_light = light
                    # waypoint index of traffic light stop line
                    line_wp_idx = temp_wp_idx

        if closest_light:
            # grab state of traffic light after getting closest light
            # call get latest light status to update state
            state = self.get_latest_light_state()

            # return light position index and state
            return line_wp_idx, state

        # if no traffic light detected or unknown state
        return -1, TrafficLight.UNKNOWN

    # -----------------------------------------------------------------------------------------------------

    def get_latest_light_state(self):
        # get the latest light state into variable
        light_status = self.classified_light_state

        # decide on stop or go
        if (light_status == TrafficLight.RED) or (light_status == TrafficLight.YELLOW):
            return TrafficLight.RED
        elif(light_status == TrafficLight.GREEN):
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN

    # -----------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
