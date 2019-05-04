
# Traffic Light Detection and Classification

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

## Annotating data
For the simulator data we used the off-the-shelf YOLO detections of traffic lights along with the OpenCV classifier to compile the dataset.
Then we manually removed false positives and corrected bad labels.

For the track data, we manually annotated the images using [LabelImg](https://github.com/tzutalin/labelImg) and [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark).

## Training
We used pre-trained networks to accelerate training and increase performance.
Two networks were used: YOLOv3 and X.
The final solution used Y.

## YOLOv3




# References

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


## darknet_ros
https://github.com/leggedrobotics/darknet_ros

## prepare dataset for YOLO
https://medium.com/@manivannan_data/how-to-train-yolov3-to-detect-custom-objects-ccbcafeb13d2
