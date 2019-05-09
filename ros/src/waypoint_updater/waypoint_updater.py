#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
# use for search nearest waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

It uses the status of traffic lights to bring the car to a stop at the stop line position.

'''

LOOKAHEAD_WPS = 100 # Number of waypoints to publish.
MAX_DECEL = 0.6 #

# SLOW MODEL = 100, 0.6
# FAST MODEL =

#-----------------------------------------------------------------------------------------------------

class WaypointUpdater(object):
    def __init__(self):

        # create ros node with unique name: waypoint_updater
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        # variable to hold the current planned trajectory
        self.final_lane = None

        #self.time_to_stop_line = None
        self.stopline_wp_idx = -1

        self.current_velocity = None

        self.prev_idx = None

        # variable to understand if classifier is ready
        self.classifier_OK = False
        # variables to understand system operation readiness state
        self.run_state = False
        self.run_status = -1
        self.prev_run_status = -1

        # wait until subscibers are ready
        rospy.wait_for_message('/current_pose', PoseStamped)
        rospy.wait_for_message('/base_waypoints', Lane)
        #rospy.wait_for_message('/traffic_waypoint', Int32)

        # subscribe to get current localised position of vehicle
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        # subscriber to get base waypoints of track
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        # subscriber to get traffic light information from simulator
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        # subscriber to get current vehicle velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        # subscribe to message to understand if classifier is running
        rospy.Subscriber('/classifier_status', Bool, self.classify_status_cb, queue_size=1)

        # publishes vehicle target trajectory as set of waypoints with speed profile
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=10)

        # define loop to give control over the publishing frequency
        self.loop()

        rospy.spin()

    # -----------------------------------------------------------------------------------------------------

    def loop(self):
      # set loop to 50Hz - waypoint_follower from autoware set to 30Hz
      rate = rospy.Rate(50)
      # loop while ros is running
      while not rospy.is_shutdown():

          # check if pose, track waypoints and waypoint tree are all ready in system
          if self.pose and self.base_waypoints and self.waypoint_tree and self.classifier_OK:

              # Inform that Vehicle is not ready for autonomous mode
              if not (self.run_state):
                  rospy.loginfo('Vehicle Ready: Pose:OK Map:OK Map Search:OK Classifier:OK')
                  self.run_state = True

              # call function to publish target trajectory waypoints
              self.publish_waypoints()

          else:
              self.run_state = False

          if not (self.run_state):

              # diagnose system to understand all parts are operating correctly
              position_status = 1 if self.pose else 0
              map_status = 1 if self.base_waypoints else 0
              tree_status = 1 if self.waypoint_tree else 0
              classify_status = 1 if self.classifier_OK else 0

              self.run_status = position_status + map_status + tree_status + classify_status

              if not (self.run_status == self.prev_run_status):
                  # diagnosis message to understand whcih parts of system are operating to enable autonomous driving
                  rospy.loginfo('Vehicle Ready State: Pose:{} Map:{} Map Search:{} Classifier:{}'.format(position_status, map_status, tree_status, classify_status))
                  self.prev_run_status = self.run_status

          rate.sleep()

      self.run_state = False

    # -----------------------------------------------------------------------------------------------------

    # callback for current_pose msg
    def pose_cb(self, msg):
        # store current pose ~50Hz
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
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # populate the waypoint tree variable with the points passed from
            self.waypoint_tree = KDTree(np.array(self.waypoints_2d))

    # -----------------------------------------------------------------------------------------------------

    # callback for current velocity msg
    def current_velocity_cb(self, msg):
        # pass msg into current_velocity variable
        self.current_velocity = msg.twist.linear.x

    # -----------------------------------------------------------------------------------------------------

    # callback for traffic_waypoint msg
    def traffic_cb(self, msg):
        # pass received message into variable stopline
        # get waypoint of next red light
        self.stopline_wp_idx = msg.data

        if not (self.prev_idx == self.stopline_wp_idx):
            self.prev_idx = self.stopline_wp_idx
            rospy.loginfo('Next Traffic light Waypoint Traffic: {}'.format(msg.data))

    # -----------------------------------------------------------------------------------------------------

    # callback for traffic_waypoint msg
    def classify_status_cb(self, msg):
        # classifier running flag
        if (msg.data == 1):
            self.classifier_OK = True
        else:
            self.classifier_OK = False

    # -----------------------------------------------------------------------------------------------------

    # main function in loop to publish trajectory waypoints
    def publish_waypoints(self):
        # build a target trajectory for vehicle to follow
        final_lane = self.generate_lane()
        # populate publisher with built trajectory
        self.final_waypoints_pub.publish(final_lane)

    # -----------------------------------------------------------------------------------------------------

    # function to take all track waypoints and generate short target vehicle trajectory
    def generate_lane(self):
        # create variable of Lane msg type
        lane = Lane()

        # localise ego to nearest waypoint
        closest_idx = self.get_closest_waypoint_idx()
        # Set end of planned trajectory based on current position and LOOKAHEAD variable
        farthest_idx = closest_idx + LOOKAHEAD_WPS

        # create a set of waypoints to represent trajectory from current position to lookahead position
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # if there is no stop line information or further than LOOKAHEAD range
        if ((self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx)):
            # just stay with base waypoints and their velocities
            return lane
        # if traffic lights are close
        else:
            # update the speed profile of waypoints to react to stop line
            lane.waypoints = self.decelerate_waypoints(lane.waypoints, closest_idx)
            # publish new trajectory waypoints
            return lane

    # -----------------------------------------------------------------------------------------------------

    # function to find closest waypoint to current position
    def get_closest_waypoint_idx(self):

      # set variables x,y to current vehicle position
      x = self.pose.pose.position.x
      y = self.pose.pose.position.y

      # store current position into tuple
      target_coordinate = np.array([x, y])

      # return closest position in track waupoints plus the index
      closest_idx = self.waypoint_tree.query(target_coordinate, 1)[1]
              
      # check if closest waypoint is ahead or behind vehicle using equation for hyperplane through closest coords
      # get coordinates of closest index from base track points and cast as numpy array
      cl_vect = np.array(self.waypoints_2d[closest_idx])
      # get coordinates of previous index in base track points
      prev_vect = np.array(self.waypoints_2d[closest_idx-1])
      # pos vect can be in front or behind hyper plane x,y is current position and hyperplane sits on this position
      pos_vect = np.array([x, y])
      
      # check if dot product if positive to see if in front of car
      # if pos_vect is in front of hyper plane: returns negative value; if behind returns positive value
      val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
      
      # if waypoint is behind/positive then take closest waypoint plus 1
      if val > 0:
          # update closest point to next waypoint
          closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

      # return closest waypoint index
      return closest_idx

    # -----------------------------------------------------------------------------------------------------

    # function to modify waypoint target speed
    def decelerate_waypoints(self, waypoints, closest_idx):

        # update waypoints velocity value (twist.linear.x) to target velocity
        # create new array to hold modified waypoints - avoid changing original
        temp = []
        # loop through all passed in waypoints
        for i, wp in enumerate(waypoints):
            # create waypoint message type
            p = Waypoint()
            # set pose to same as base waypoint pose
            p.pose = wp.pose

            # find number of waypoints between current position and stopline waypoint
            stop_idx = max(self.stopline_wp_idx - closest_idx - 4, 0) # three waypoints back to move measure point to front of car - stop at stop line
            # for each waypoint in shortened list calculate distance from stop line - linear piece wise distance
            # distance returns 0 if is bigger than stop index
            dist = self.distance(waypoints, i, stop_idx)
            # large velocity if large distance so as distance reduces velocity reduces
            # sqrt is severe at end so to get a faster deceleration
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # if velocity <1 set to 0 to stop
            if vel < 1.:
                vel = 0.0

            # select minimum as sqrt is large if distance is large so limit to waypoint speed as max
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            # add waypoint to new waypoint list
            temp.append(p)

        return temp

    # -----------------------------------------------------------------------------------------------------

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # -----------------------------------------------------------------------------------------------------

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # -----------------------------------------------------------------------------------------------------

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # -----------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        print("node launched")
        WaypointUpdater()
        
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
