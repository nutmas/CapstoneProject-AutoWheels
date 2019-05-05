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

The first approach was to use a off-the-shelf detector (YOLO) to detect the traffic lights.
Simulator detections were very good.
For OpenCV based classifier was developed to work over the YOLO detections.
This system performed well on simulator, but not on track data.
Off-the-shelf detection on the track data (from provided ROS bag) was poor.
But OpenCV classification was also much harder.
It became clear that
 1) we would need to train the detector with track data; and
 2) an OpenCV classifier would not work on track data.

We moved on to train networks with both simulator and track data.
We trained the networks to detect new classes.
Instead of detecting `traffic light` they would detect either `red`, `green` or `yellow`.
This model would integrate in one what was once a seperate detector and classifier.

### Annotating data
For the simulator data we used the off-the-shelf YOLO detections of traffic lights along with the OpenCV classifier to compile the dataset.
Then we manually removed false positives and corrected bad labels.

For the track data, we manually annotated the images using [LabelImg](https://github.com/tzutalin/labelImg) and [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark).

### Training
We used pre-trained networks to accelerate training and increase performance.
Two networks were used: YOLOv3 and X.
The final solution used Y.

### YOLOv3




### References

Annotation:
 - LabelImg | https://github.com/tzutalin/labelImg
 - Yolo_mark | https://github.com/AlexeyAB/Yolo_mark

Detection and Classification:
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


### darknet_ros
https://github.com/leggedrobotics/darknet_ros

### prepare dataset for YOLO
https://medium.com/@manivannan_data/how-to-train-yolov3-to-detect-custom-objects-ccbcafeb13d2


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
