#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

# use for search nearest waypoint
from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.base_waypoints = None
        self.waypoint_tree = None
        self.waypoints_2d = None

        # wait until subscibers are ready
        rospy.wait_for_message('/current_pose', PoseStamped)
        rospy.wait_for_message('/base_waypoints', Lane)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):

        self.base_waypoints = waypoints

        # store waypoints in the object
        # latched subscriber - base waypoints never change so shuld only be stored once
        print("traffic callback launched")

        # print("waypoint size: {}".format(len(waypoints)))
        # if not used to make sure self.waypoints_2d is initialised before the subscriber is
        if not self.waypoints_2d:
            # convert waypoints to 2D coordinates for each waypoint using list comprehension
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            # self.waypoint_tree = KDTree(zip(waypoints.waypoint.pose.pose.position.x.ravel(), waypoints.waypoint.pose.pose.position.y.ravel()))
            self.waypoint_tree = KDTree(np.array(self.waypoints_2d))
            # for i in (waypoints.waypoints):
            # print waypoints.waypoints.pose.position.x

            #print "traffic light waypoint tree list:"
            #print self.waypoint_tree.data

            #ptsTofind = np.array([909.5, 1128])

            #result = self.waypoint_tree.query(ptsTofind, 1)[1]
            #print "traffic light result:::: "
            #print result


        pass









    def traffic_cb(self, msg):
        self.lights = msg.lights

        # publish light message when not using classifier
        light_wp, state = self.process_traffic_lights()

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
            # set light waypint as stop line if light state is RED
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            # publish message with latest traffic light state
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        # if state is same and threshol is not yet reached - keep publishing previous light state
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        # increment state counter as same state witnessed again
        self.state_count += 1



    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
            """
        # find closest waypoint index
        target_coordinate = np.array([x, y])

        closest_idx = self.waypoint_tree.query(target_coordinate, 1)[1]

        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        # for testing return light state
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
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


            #print "pose closest car point"
            #print car_wp_idx

            # TODO find the closest visible traffic light (if one exists)
            # iterate through the whole list of traffic lights
            diff = len(self.base_waypoints.waypoints)
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
            state = self.get_light_state(closest_light)

            # return light position index and state
            return line_wp_idx, state

        # if no traffic light detected or unknown state
        return -1, TrafficLight.UNKNOWN





        """
        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN """

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
