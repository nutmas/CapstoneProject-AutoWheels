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
The simulator detections for the YOLO bounding boxes were very good. We had hoped to combine these detections with a simple OpenCV approach to light classifications. This end to end system worked very well in simulation, but not on track data. The off-the-shelf YOLO detection on the track data (from provided ROS bag) was poor. The OpenCV approach had an even more difficult time with the real track data due to the bright lighting conditions. We decided it would be better and more reliable to learn a model to classify the light colors rather than try to account for the brightness differences in OpenCV. It was clear that we would need to train a detector and classifier with both simulation and real track data. Along these lines we had two distinct approaches: 

1. Train two models, one for traffic light bounding box detection and another for traffic light color classification. These were the YOLO and Darknet models respectively and fed inputs into one another.

2. Create a single model the integrated detection and light classification.

### Annotating data
For the simulator data we used the off-the-shelf YOLO detections of traffic lights along and the OpenCV classifier to compile a training dataset. We manually exammined this training set correcting any false positives or bad labels. For generating a training set from track data, we manually annotated the images using [LabelImg](https://github.com/tzutalin/labelImg) and [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark).

### Training
We used pre-trained networks to accelerate training and increase performance.
Two networks were used: YOLOv3 and X.
The final solution used Y.

### OpenCV
The OpenCV approach included in tl_classifier.py further cropped and resized the input images and converted them from BRG to HSV. The V componenet of the image was binary thresholded and the image was split into top, middle and bottom components corresponding to the red, yellow and green lights. The section with the largest greater than zero count determined the light color. This was inspired by the reference blog included below.

### YOLOv3 and Darknet
TODO(Volker, Diogo): Discuss your two part training approach using YOLO and Darknet.

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

## References

#### Links
* [LabelImg](https://github.com/tzutalin/labelImg)
* [OpenCV Traffic Light Detection Blog](https://qtmbits.com/traffic-light-classifier-using-python-and-opencv/)
* [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark)
* [Preparing dataset for YOLO Blog](https://medium.com/@manivannan_data/how-to-train-yolov3-to-detect-custom-objects-ccbcafeb13d2)
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros)

#### Article Citations
@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}

@article{opencv_library,
    author = {Bradski, G.},
    citeulike-article-id = {2236121},
    journal = {Dr. Dobb's Journal of Software Tools},
    keywords = {bibtex-import},
    posted-at = {2008-01-15 19:21:54},
    priority = {4},
    title = {{The OpenCV Library}},
    year = {2000}
}


# Usage Instructions

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

## Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

## Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

## Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

## Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

## Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
