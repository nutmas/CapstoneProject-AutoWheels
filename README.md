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

1. The YOLO object detection system, pre-trained on the COCO dataset with 80 classes, including traffic lights.
2. Use a pre-trained model using the Tensorflow framework and expand it to not only detect traffic lights but also classify their state.

Although both approaches were explored and developed, the YOLO system was used for the final solution.

### Annotating data
For the simulator data we used the off-the-shelf YOLO detections of traffic lights along and the OpenCV classifier to compile a training dataset. We manually exammined this training set correcting any false positives or bad labels. For generating a training set from track data, we manually annotated the images using [LabelImg](https://github.com/tzutalin/labelImg) and [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark).

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

### End to End Classification using Tensorflow

#### Development Overview 

An end-to-end approach in the traffic light detection context equates to passing the classifier an image; it then identifies the location in the scene
and also categorises the traffic light state as RED, YELLOW or GREEN.

To achieve this I decided to develop a network model by retraining an existing model from the Tensorflow [model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)

The models selected were deemed suitable for the traffic light task, based on performance and output:  
**faster_rcnn_inception_v2_coco** Speed: 60ms, Quality: 28mAP, Output: Boxes  
**faster_rcnn_resnet101_coco** Speed: 106ms, Quality: 32mAP, Output: Boxes

The following process was utilised to retrain the models to enable them to classify traffic lights in the simulator.

- Drive around simulator track and log images received from camera on rostopic /image_color. To get a range of traffic light conditions 3 Laps of track data was gathered.
- A dataset was compiled using [labelimg](https://github.com/tzutalin/labelImg). Bounding boxes were drawn around the front facing traffic lights, and labelled as RED, YELLOW, GREEN or UNKOWN. Images with no traffic lights were not labelled.
- The Object Detection libraries in Tensorflow v1.12 were required to enable re-training of the models. The dataset was converted to a tensorflow 'record' to proceed with training.
- The basic configuration for each model in the training setup is:
    - Inception v1: Epoch: 2000 Input Dimensions: min:600 max:800
    - Inception v2: Epoch: 20000 Input Dimensions: min:600 max:800
    - Resnet: Epoch: 80000 Input Dimensions: min:600 max:800
- Training the models was performed using the scripts available in the Tensorflow Object library.
- I created python-notebook pipeline to test each model against a set of images which the model had not seen during training. The notebook painted bounding boxes on each image, providing the classification and confidence. 500 images passed through produced the results for Inception v2 are shown in this [Video](https://www.youtube.com/watch?v=1QT6ahoyVDY&t=124s)
- After successful static image evaluation all models were frozen; For compatibility with Udacity environment freezing was performed using Tensorflow v1.4.
- The frozen models were integrated into the [`tl_classifier.py`](https://github.com/nutmas/CapstoneProject-AutoWheels/blob/TensorBranch/ros/src/tl_detector/light_classification/tl_classifier.py) node of the pipeline.  
    - From ROS camera image is received by 'tl_detector.py' and passed into a shared lockable variable.
    - The function `get_classification()` is ran in a parallel thread to process the image and utilise the classifier. This avoids the classifier impacting on the ROS processing its other tasks.
    - The classifier processes the image and returns the detection and classification results.
    - The array of classification scores for each traffic light detection are evaluated and highest confidence classification is taken as the result to pass back to [`tl_detector.py`](https://github.com/nutmas/CapstoneProject-AutoWheels/blob/TensorBranch/ros/src/tl_detector/tl_detector.py)
    - In Parallel to classification thread, the [`tl_detector.py`](https://github.com/nutmas/CapstoneProject-AutoWheels/blob/TensorBranch/ros/src/tl_detector/tl_detector.py)function `run_main()` continuously calculates the nearest traffic light based on current pose, to understand the distance to next stop line. When a position and classification are aligned, the node will only output a waypoint representing distance to stop line, if the traffic light is RED or YELLOW.
    - The [`waypoint_updater.py`](https://github.com/nutmas/CapstoneProject-AutoWheels/blob/TensorBranch/ros/src/waypoint_updater/waypoint_updater.py) receives the stop line waypoint and will control the vehicle to bring it to a stop at the stop line position. Once a green light is present the waypoint is removed and the vehicle accelerates to the set speed.

#### Performance Evaluation 

- Inception v1 model has lower accuracy but runs faster producing results of ~330ms per classification (On 1050Ti GPU). However this required more classification outputs to establish a confirmed traffic light state.
- Inception v2 model has very high accuracy but runs much slower ~1.5secs per classification (On 1050Ti GPU). This can work on a single state result.
- Both models could successfully navigate the track and obey the traffic lights. However both classifications took over 1 second to have a confirmed state. v1 would sometimes mis-classify a number of times and due to the higher state change requirements could miss a red light.
- The simulator would crash at a certain point sometimes and the styx server crash, this occurred more frequently on the v2 model. Videos showing the performance of each model are shown in the videos:
    + [inception v1 video](https://www.youtube.com/watch?v=G_5z3RUoplA)
    + [inception v2 video](https://www.youtube.com/watch?v=eRHMHTRL228&t=4s)
- I evaluated the models on a 1080Ti GPU which is similar specification to the Udacity hardware. This hardware change significantly improved the speed performance time of the classifiers. The v2 dropped from 1.5s to 650ms and maintained it quality which meant ti was a good solution for successfully navigating the simulator. The results can be seen in this [Video](https://www.youtube.com/watch?v=OaNf-dULUBw)
    
#### Conclusion for end-to-end classifier
The v1 and v2 inception models are similar size once frozen (52MB vs 55MB). However the model which ran for 10x more epoch is significantly slower but has a much higher reliability for classification. The v2 model was chosen as it could perform to the meet the requirement of the simulator track.
No real world data training or testing was performed on the classifier yet; it was therefore judged by the team that the YOLO classifier with Darknet would be more suitable for the submission.
To take this end-to-end classifier forwards it would need retraining on the real world data and have a switch in the launch file to select real world or simulator world models.

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

