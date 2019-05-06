from styx_msgs.msg import TrafficLight
import rospy

import tensorflow as tf
import numpy as np
import time

#-----------------------------------------------------------------------------------------------------

class TLClassifier(object):

    def __init__(self):

        # load trained model
        FROZEN_MODEL = r'light_classification/trained_models/ssd_inception/frozen_inference_graph.pb'

        # setup saved model
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # read in the saved model
            with tf.gfile.GFile(FROZEN_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # input to model
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            # outputs from model
            # bounding box
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # confidence level
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            # state classification
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            # number of detections in scene
            self.number_of_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.detection_graph)

# -----------------------------------------------------------------------------------------------------

    def get_classification(self, image):

        # Determines the color of the traffic light in the image

        # implement light color prediction from net model - ssd inception
        # setup model
        with self.detection_graph.as_default():

            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            input_image = np.expand_dims(image, axis=0)

            # Run inference on saved model speicfy passed in image and get results
            # log start time
            start_time = time.time()
            (boxes, scores, classes, number_of_detections) = self.sess.run([self.detection_boxes,
                                                                            self.detection_scores,
                                                                            self.detection_classes,
                                                                            self.number_of_detections],
                                                                           feed_dict={self.image_tensor: input_image})
            # log time taken to classify
            time_taken = time.time() - start_time

            # Take highest score as classification result
            # 1 = Green, 2 = Red, 3 = Yellow, 4 = Unknown
            score = scores[0][np.argmax(scores)]

            # if the confidence is more than 50%
            if score > 0.5:
                # get class with highest score
                predicted_state = classes[0][np.argmax(scores)]

                # transform to message state
                if predicted_state == 1:
                    #print("Green Light {}".format(score))
                    rospy.loginfo("Classification time: {}, result: {}(GREEN) Confidence: {}".format(time_taken, predicted_state, score))
                    return TrafficLight.GREEN
                elif predicted_state == 2:
                    #print("Red Light {}".format(score))
                    rospy.loginfo("Classification time: {}, result: {}(RED) Confidence: {}".format(time_taken, predicted_state, score))
                    return TrafficLight.RED
                elif predicted_state == 3:
                    #print("Yellow Light {}".format(score))
                    rospy.loginfo("Classification time: {}, result: {}(YELLOW) Confidence: {}".format(time_taken, predicted_state, score))
                    return TrafficLight.YELLOW

        # if can't classify then return unknown
        return TrafficLight.UNKNOWN

    # -----------------------------------------------------------------------------------------------------

    # function to clean up tensorflow session once ended
    def __exit__(self, exc_type, exc_val, exc_tb):

        self.sess.close()
        rospy.loginfo('Tensorflow session is ended')

