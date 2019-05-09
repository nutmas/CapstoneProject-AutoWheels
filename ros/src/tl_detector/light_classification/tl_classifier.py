from styx_msgs.msg import TrafficLight
import rospy

#import tensorflow as tf
import numpy as np
import time

# for darknet
import actionlib
import darknet_ros_msgs.msg as darknet_msgs

#-----------------------------------------------------------------------------------------------------

class TLClassifier(object):

    def __init__(self):
        
        # darknet implementation
        # create a darknet client to send messages to classifier
        self.darknet_client = actionlib.SimpleActionClient('/darknet_ros/check_for_objects', darknet_msgs.CheckForObjectsAction)
        # ????
        self.darknet_goal_id = 0
        
        
        # console indicator to understand status of darknet
        # rospy.logwarn('waiting for darknet action server...')
        # ready for server
        self.darknet_client.wait_for_server()

# -----------------------------------------------------------------------------------------------------

    def get_classification(self,rx_image):

        # Determines the color of the traffic light in the image
        
        # log start time
        t_start = time.time()
        # create darknet goal with received image
        goal = darknet_msgs.CheckForObjectsGoal()
        goal.image = rx_image
        goal.id = self.darknet_goal_id
        self.darknet_goal_id += 1
        
        
        
        # send goal and wait for result
        self.darknet_client.send_goal(goal)
        #rospy.logwarn('waiting for darknet action result...')
        self.darknet_client.wait_for_result()
        result = self.darknet_client.get_result()  # result is BoundingBoxes message
        
        # calculate time taken to classify
        time_taken = time.time() - t_start
        
        # traffic light states in correct order for return code 0 (red), 1(yellow), 2(green)
        colors = ['red', 'yellow', 'green']
        
        # compile valid detections
        det = [bb.Class for bb in result.bounding_boxes.bounding_boxes if bb.Class in colors]
        
        predicted_state = 4

        if len(det)== 0:
            # if not valid detections return unknown state
            return TrafficLight.UNKNOWN
        else:
            # update latest light_state
            # get the number of detections for each color and sort them
            counts = [(color, det.count(color)) for color in colors]
            counts = sorted(counts, key=lambda t: t[1])
            most_frequent = counts[-1]
            predicted_state = colors.index(most_frequent[0])

        # transform to message state
        if predicted_state == 0:
            #print("Green Light {}".format(score))
            rospy.loginfo("Classification time: {}, result: {}(RED) Confidence: {}".format(time_taken, predicted_state, most_frequent))
            return TrafficLight.RED
        elif predicted_state == 1:
            #print("Red Light {}".format(score))
            rospy.loginfo("Classification time: {}, result: {}(YELLOW) Confidence: {}".format(time_taken, predicted_state, most_frequent))
            return TrafficLight.YELLOW
        elif predicted_state == 2:
            #print("Yellow Light {}".format(score))
            rospy.loginfo("Classification time: {}, result: {}(GREEN) Confidence: {}".format(time_taken, predicted_state, most_frequent))
            return TrafficLight.GREEN

        # if can't classify then return unknown
        #return TrafficLight.UNKNOWN

    # -----------------------------------------------------------------------------------------------------

    # function to clean up tensorflow session once ended
    def __exit__(self, exc_type, exc_val, exc_tb):

        rospy.loginfo('Classifier is ended')

