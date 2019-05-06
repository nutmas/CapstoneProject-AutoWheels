# Programming a Real Self Driving Car
The goal of the final Udacity Capstone project was to program a real self driving car using ROS. Much of the developement of the car was done the Udacity Simulator, but the final goal of the project was to run the code in real life on Carla, Udacity's self-driving car.

## Meet Team AutoWheels
Team AutoWheels has five memembers. Below are their names, email addresses and slack handles.

Team Member Name | Email Address | Slack Handle 
------------ | ------------- | -------------
Diogo Silva (Team Lead) | akins.daos+selfdriving@gmail.com | @diogoaos
Volker van Aken | volker.van.aken@gmail.com | @Volker
Andreea Patachi | patachiandreea@yahoo.com | @Andreea	
Stephen Nutman | stephen_nutman@outlook.com | @Steve
Alexander Meade | alexander.n.meade@gmail.com | @ameade 


## Traffic Light Detection and Classification

The first approach was to use a off-the-shelf detector (YOLO) to detect the bounding boxes of traffic lights.
The simulator detections for the YOLO bounding boxes were very good.
We had hoped to combine these detections with a simple OpenCV approach to light classifications.
This end to end system worked very well in simulation, but not on track data.
The off-the-shelf YOLO detection on the track data (from provided ROS bag) was poor.
The OpenCV approach had an even more difficult time with the real track data due to the bright lighting conditions.
We decided it would be better and more reliable to learn a model to classify the light colors rather than try to account for the brightness differences in OpenCV.
It was clear that we would need to train a detector and classifier with both simulation and real track data.
Along these lines we had two distinct approaches: 

1. Train two models, one for traffic light bounding box detection and another for traffic light color classification. These were the YOLO and Darknet models respectively and fed inputs into one another.

2. Create a single model the integrated detection and light classification.

### Annotating data
For the simulator data we used the off-the-shelf YOLO detections of traffic lights along and the OpenCV classifier to compile a training dataset. We manually exammined this training set correcting any false positives or bad labels. For generating a training set from track data, we manually annotated the images using [LabelImg](https://github.com/tzutalin/labelImg) and [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark).

### Training
We used pre-trained networks to accelerate training and increase performance.
Two networks were used: YOLOv3 and X.
The final solution used Y.

### OpenCV
The OpenCV approach included in tl_classifier.py further cropped and resized the input images and converted them from BRG to HSV. The V componenet of the image was binary thresholded and the image was split into top, middle and bottom components corresponding to the red, yellow and green lights. The section with the largest greater than zero count determined the light color. This was inspired by the reference [blog](https://qtmbits.com/traffic-light-classifier-using-python-and-opencv/) included below. We also tried experiementing with Hough circles in OpenCV, but the results were not consistent in that a circle was not always detected.

### YOLOv3 and darknet_ros
#### YOLOv3
YOLO (You Only Look Once) is known to be a fast algorithm for object detection.
YOLOv2 improved on the original algorithm by being faster and more accurate.
YOLOv3 sacrificed speed for better accuracy.
While slower, the authors report 30 FPS in a Titan X, similar to the GPU installed in CARLA.
Furthermore, 30FPS is a processing speed higher then the publish frequency for the image feed in both the simulator and provided ROS bags.
Thus YOLOv3, pretrained on COCO dataset, was a good fit for this project.
Furthermore, a ROS package (_darknet_ros_) that made YOLO easily available in the the environment was available, which also sped up development.

YOLOv3 is built on a 53 layer network trained on imagenet (_darknet-53_), on top of which 53 more layers are added for object detection.
Contrary to its predecessor, YOLOv3 uses both skip connections and upsampling layers, resulting in a fully convolutional model.
Detection is done at 3 different places in the network, using features at different scales.
This feature of the model helps with detection of small objects.

A full architecture of the underlying model can be observed in the image below.

![YOLOv3 underlying architecture diagram](yolov3.png)
YOLOv3 underlying architecture (taken from [here](https://towardsdatascience.com/yolo-v3-object-detection-53fb7d3bfe6b)).
#### darknet_ros and tl_detector logic
After training the model with the data for this project, the integration was done through the darknet_ros package and some simple logic in `tl_detector.py`.

darknet_ros already has a ROS action server implemented.
In `tl_detector.py`, we implemented an action client.
Each time an image was received, a Action Goal containing that image was sent to darknet_ros.
The response was a Action Result containing the object detections.

Since multiple traffic lights could be detected, the final state was given by the traffic light state that was most prevalent.
For example, if 2 red lights and 1 yellow were detected, the final state would be corresponding to the red light.
This logic worked well.
Videos of the performance can be seen in the Results section.

### End to End Net

TODO(Steve): Discuss your end to end model approach for cropping and classification.

## Self Driving Car Control

A lot of the control and waypoint following for our self-driving car was re-used from the walk-through code. The walk-through provided a great starting point. That being said, there were some key changes that improved performance that are worth highlighting below.

* Update the waypoint follower so that the `PurePursuit::verifyFollowing()` always returned false. This helped prevent the car from wandering in the lane. This is because previously the function calculated if the car was "close enough" to the waypoints in which case it would follow the previous control inputs resulting. This change made the car always follow the most recent inputs.

* Updated the waypoint updater `decelerate_waypoints()` function so that it accepted the stop waypoint index as an argument. This prevented the waypoint node from crashing randomly due to an edge case where the code passed the `stop_wp_idx_temp >= farthest_idx` check in generate lane and then received a /traffic_waypoint update in the middle of calculating the deceleration waypoints resulting in an out of bounds exception at the worst. At the best it resulted in decelerion waypoints calculated using multiple stop locations.

* Updated the twist controller steering PID parameters to improve performance around the curves. The final values of were the following (KP = 0.9, KI = 0.007, KD = 0.2, MN = 0.0, MX = 0.2).

* Updated the total vehicle mass calculation in the twist controller to include the mass contributed by the current fuel levels. While this was impossible to test in simulation it should help provide more accurate braking torques when decelerating the vehicle on the track.

* Changed the idle vehicle stop brake torque within twist controller to a ros server param `~stop_brake_torque`. This allowed us to more easily adjusted the Nm torque applied in simulation and real life to 400 Nm and 700 Nm respectively.

### Simulation Inconsistency

Our team struggled a lot with reproducing controller results in simulation. This is because the Udacity workspace's, the VMs and our personal machines all had different performance specs. If the machine ran slower for any period of time the car would stop following the waypoints and drive out of bounds. This was especially prevalent in the Udacity workspace environemnt. These students experienced similar issues [here](https://knowledge.udacity.com/questions/39086).

## Results
The performance of solution in one lap of the simulator was recorded and published on Youtube (https://youtu.be/p0Jgx_Gc7sg).
The video was accelerated 3x.
YOLO was trained on around 4000 images for 7000 epochs.
The same model was tested on track images. A video was recorded and published on Youtube (https://youtu.be/Cov7dlHjFko).

## References

#### Links
* [LabelImg](https://github.com/tzutalin/labelImg)
* [OpenCV Traffic Light Detection Blog](https://qtmbits.com/traffic-light-classifier-using-python-and-opencv/)
* [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark)
* [Preparing dataset for YOLO Blog](https://medium.com/@manivannan_data/how-to-train-yolov3-to-detect-custom-objects-ccbcafeb13d2)
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)

#### Article Citations
Redmon, J. and Farhadi, A., 2018. Yolov3: An incremental improvement. arXiv preprint arXiv:1804.02767. [YOLOv3 article](https://pjreddie.com/media/files/papers/YOLOv3.pdf)

Bradski, G., The OpenCV Library. Dr. Dobb's Journal of Software Tools , (2000). [OpenCV](https://opencv.org/)

