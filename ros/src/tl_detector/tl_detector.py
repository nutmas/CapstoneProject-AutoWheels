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
import os
import time


import actionlib
import darknet_ros_msgs.msg as darknet_msgs

# use for search nearest waypoint
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        #Added
        self.waypoints_2d = None
        self.waypoint_tree = None

        # wait until subscibers are ready
        rospy.logwarn('waiting for current pose message...')
        rospy.wait_for_message('/current_pose', PoseStamped)
        rospy.logwarn('waiting for base waypoints...')
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # darknet 
        self.darknet_client = actionlib.SimpleActionClient('/darknet_ros/check_for_objects', darknet_msgs.CheckForObjectsAction)
        self.darknet_goal_id = 0

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # wait for darknet classifier to bootup
        rospy.logwarn('waiting for darknet action server...')
        self.darknet_client.wait_for_server()
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

#         UNCOMMENT THE FOLLOWING LINES TO TEST SIMULATION WITHOUT CAMERA
#         light_wp, state = self.process_traffic_lights()

#         '''
#         Publish upcoming red lights at camera frequency.
#         Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
#         of times till we start using it. Otherwise the previous stable state is
#         used.
#         '''
#         if self.state != state:
#             self.state_count = 0
#             self.state = state
#         elif self.state_count >= STATE_COUNT_THRESHOLD:
#             self.last_state = self.state
#             light_wp = light_wp if state == TrafficLight.RED else -1
#             self.last_wp = light_wp
#             self.upcoming_red_light_pub.publish(Int32(light_wp))
#         else:
#             self.upcoming_red_light_pub.publish(Int32(self.last_wp))
#         self.state_count += 1

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.logwarn('current state is: {}'.format(state))
        

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
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx
        
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
#         if(self.has_image):
#             cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
#             uniq = 0
#             path = '/data/sdc/CarND-Capstone/ros/image_color/img{:04d}.png'.format(uniq)
#             while os.path.exists(path):
#                 path = '/data/sdc/CarND-Capstone/ros/image_color/img{:04d}.png'.format(uniq)
#                 uniq += 1
#             cv2.imwrite(path, cv_image) 

        #Get classification
        #return self.light_classifier.get_classification(cv_image)

        # if there is no image, return UNKNOWN
        if not self.has_image:
            return 4

        t_start = time.time()
        # create darknet goal with received image
        goal = darknet_msgs.CheckForObjectsGoal()
        goal.image = self.camera_image
        goal.id = self.darknet_goal_id
        self.darknet_goal_id += 1

        # send goal and wait for result
        self.darknet_client.send_goal(goal)
        #rospy.logwarn('waiting for darknet action result...')
        self.darknet_client.wait_for_result()
        result = self.darknet_client.get_result()  # result is BoundingBoxes message

        t_total = time.time() - t_start
        rospy.logwarn('classification time = {}'.format(t_total))

        # traffic light states in correct order for return code 0 (red), 1(yellow), 2(green)
        colors = ['red', 'yellow', 'green']

        # compile valid detections
        det = [bb.Class for bb in result.bounding_boxes.bounding_boxes if bb.Class in colors]

        # if no valid detections, unknown state
        if len(det) == 0:
            return 4 # UNKNOWN

        # get the number of detections for each color and sort them
        counts = [(color, det.count(color)) for color in colors]
        counts = sorted(counts, key=lambda t: t[1])
        most_frequent = counts[-1]
        state_code = colors.index(most_frequent[0])

        return state_code
        
        # for testing return light state
        # return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = None
        

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints and self.waypoint_tree):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i,light in enumerate(self.lights):
                #Get stop line waypont idx
                line = stop_line_positions[i]

                temp_wp_idx = self.get_closest_waypoint(line[0],line[1])
                
                #Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx

                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx


        if closest_light:
            # grab state of traffic light after getting closest light
            state = self.get_light_state(closest_light)
            # return light position index and state
            return line_wp_idx, state
        # if no traffic light detected or unknown state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
