import cv2
import numpy as np
# from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        # Size to crop all input images to for both height and width.
        self.crop_size = 64
        self.top_margin = 4
        self.side_margin = 10
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image.

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight).

        """

        # Uncomment to show original image.
        cv2.imshow("Image", image)
        cv2.waitKey()

        # Crop the input image to a consistent shape.
        image = cv2.resize(image, (self.crop_size, self.crop_size))

        # Convert the image to HSV and extract the value channel to get the brightest colors.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv)

        # Threshold image to binary to get only colored pixels.
        retval, thresholded = cv2.threshold(value,  120, 255, cv2.THRESH_BINARY)

        # Uncomment to show thresholded value channel of HSV image.
        cv2.imshow("Thresh", thresholded)
        cv2.waitKey(0)

        # Slice the traffic light to middle top and bottom.
        top_red = thresholded[self.top_margin: self.crop_size / 3,
                              self.side_margin: self.crop_size - self.side_margin]
        middle_yellow = thresholded[self.crop_size / 3: self.crop_size * 2 / 3 ,
                                    self.side_margin: self.crop_size - self.side_margin]
        bottom_green = thresholded[self.crop_size * 2 / 3: self.crop_size - self.top_margin,
                                   self.side_margin :self.crop_size-self.side_margin ]

        # Uncomment to show the binary traffic slices.
        # cv2.imshow("Red", top_red)
        # cv2.waitKey(0)
        # cv2.imshow("Yellow", middle_yellow)
        # cv2.waitKey(0)
        # cv2.imshow("Green", bottom_green)
        # cv2.waitKey(0)

        # Count the non zero pixels in each traffic light slice.
        count_red = cv2.countNonZero(top_red)
        count_yellow = cv2.countNonZero(middle_yellow)
        count_green = cv2.countNonZero(bottom_green)
        counts = [count_red, count_yellow, count_green]

        # Decide which light color it is by identifying the segment with the most thresholded pixels.
        max_index = np.argmax(counts)
        if max_index == 0:
            return 0 # RED
        elif max_index == 1:
            return 1 # YELLOW
        elif max_index == 2:
            return 2 # GREEN
        else:
            return 4 # UNKNOWN

