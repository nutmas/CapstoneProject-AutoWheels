#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
# use for search nearest waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.8 #

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.base_lane = None
        self.stopline_wp_idx = -1

        # wait until subscibers are ready

        rospy.wait_for_message('/current_pose', PoseStamped)
        rospy.wait_for_message('/base_waypoints', Lane)
        rospy.wait_for_message('/traffic_waypoint', Int32)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # subscriber to get base waypoints ahead of vehicle
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # subscriber to get traffic light information
        rospy.Subscriber('traffic_waypoint', Int32, self.traffic_cb)
        
        


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        
        # define loop to give control over the publishing frequency
        self.loop()

        #rospy.spin()
    def loop(self):
      # set loop to 50Hz - waypoint_follower from autoware set to 30Hz
      rate = rospy.Rate(50)
      print("loop launched")
      while not rospy.is_shutdown():

          # check if pose & base_lane are better options
          if self.pose and self.base_waypoints and self.waypoint_tree:
              self.new_publish_waypoints()

          # if pose & base is avaialable.. .and waypints actually exist
          #if self.pose and self.base_waypoints and self.waypoint_tree:
              #print("if is true launched")
              #Get closest waypoint
              #closest_waypoint_idx = self.get_closest_waypoint_idx()
              #self.publish_waypoints(closest_waypoint_idx)
          rate.sleep()
          
    def get_closest_waypoint_idx(self):
      
      x = self.pose.pose.position.x
      y = self.pose.pose.position.y

      target_coordinate = np.array([x, y])
      
      #print("about to query x:{} y:{}".format(x,y))
      # return closest position plus the index
      closest_idx = self.waypoint_tree.query(target_coordinate, 1)[1]
      #closest_idx = self.waypoint_tree.query(target_coordinate, k=2)

      #x, y = self.waypoint_tree.query(source_latlon, k=2)
              
      #check if closest waypoint is ahead or behind vehicle
      closest_coord = self.waypoints_2d[closest_idx]
      prev_coord = self.waypoints_2d[closest_idx-1]
        
      #equation for hyperplane through closest coords
      # closest way point
      cl_vect = np.array(closest_coord)
      # previous waypoint
      prev_vect = np.array(prev_coord)
      # pos vect can be in front or behind hyper plane x,y is current position and hyperplane sits on this position
      pos_vect = np.array([x, y])
      
      # check if dot product if positive to see if in front of car
      # if pos_vect is in front of hyper plane or behind
      # will return a positive value if behind the car
      val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
      
      # if waypoint is behind/positive then take closest waypoint plus 1
      if val > 0:
          closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
      return closest_idx


    def publish_waypoints(self, closest_idx):

        lane = Lane()
        # header not used so not so important to copy across
        lane.header = self.base_waypoints.header
        # take closest point and lookahead to set current points
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

        self.final_waypoints_pub.publish(lane)

    def new_publish_waypoints(self):
        final_lane = self.generate_lane()
        self.base_lane = True
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        # create a Lane msg
        lane = Lane()

        # header not used so not so important to copy across
        lane.header = self.base_waypoints.header

        # take waypoints and update their velocity based on distance to traffic lights

        # localise ego to nearest waypoint
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        # create a set of waypoints from current position to lookahead position
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        #print ("Stopline index: {}".format(self.stopline_wp_idx))
        current_velocity = self.base_waypoints.waypoints[closest_idx].twist.twist.linear.x


        #print ("waypoints Velocity: {}".format(current_velocity))

        # if there is no trafifc light information or too far away
        if ((self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx)):
            # just stay with base waypoints  and their velocities
            #lane.waypoints = base_waypoints
            return lane
        # if traffic lights are close
        else:
            # update the speed profile of waypoints to react to stop line
            lane.waypoints = self.decelerate_waypoints(lane.waypoints, closest_idx)

        return lane

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
            # -2 as centre of car is pose position
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # two waypoints back to get front of car to stop at stop line
            # for each waypoint in shortened list calculate distance from stop line - linear piece wise distance
            # distance returns 0 if is bigger than stop index
            dist = self.distance(waypoints, i, stop_idx)
            # large velocity if large distance so as distance reduces velocity reduces
            # sqrt is severe at end so could use a linear deceleration with a constant
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # if velocity <1 set to 0 to stop
            if vel < 1.0:
                vel = 0.0

            # select minimum as sqrt is large if distance is large so limit to waypoint speed as max
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            # add waypoint to new waypoint list
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # store current pose ~50Hz
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # store waypoints in the object
        # latched subscriber - base waypoints never change so should only be stored once
        self.base_waypoints = waypoints
        #print("callback launched")

        #print("waypoint size: {}".format(len(waypoints)))
        # if not used to make sure self.waypoints_2d is initialised before the subscriber is
        if not self.waypoints_2d:
            # convert waypoints to 2D coordinates for each waypoint using list comprehension
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        #self.waypoint_tree = KDTree(zip(waypoints.waypoint.pose.pose.position.x.ravel(), waypoints.waypoint.pose.pose.position.y.ravel()))
            self.waypoint_tree = KDTree(np.array(self.waypoints_2d))
        #for i in (waypoints.waypoints):
            #print waypoints.waypoints.pose.position.x

            #print "waypoint tree list:"
            #print self.waypoint_tree.data

            #ptsTofind = np.array([909.5, 1128])

            #result = self.waypoint_tree.query(ptsTofind, 1)[1]
            #print "result:::: "
            #print result

        #self.waypoint_tree = KDTree(self.waypoints_2d)

        #self.waypoint_tree.query

        #source_latlon = 1131.23, 1183.27

        #x, y = self.waypoint_tree.query(source_latlon, k=2)
        #print("way 2d: {} {}".format(x, y))


            #for x,y in self.waypoint_tree:
                #self.waypoints_2d[[0]] = waypoint.pose.pose.position.x
                #self.waypoints_2d[[1]] = waypoint.pose.pose.position.y
                #print(waypoint.pose.pose.position.x)
                #print(waypoint.pose.pose.position.y)
                #self.waypoints_2d.append([[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]])
                #print("way 2d: {} {}".format(x,y))




            # use KDTree to lookup closest point in space log_n search
            #print("waypoint tree callback")

            #print("waypoint size: {}".format(self.waypoint_tree.size()))


        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # pass received message into variable
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist






if __name__ == '__main__':
    try:
        print("node launched")
        WaypointUpdater()
        
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
