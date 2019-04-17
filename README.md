# vision_pipeline
A Vision Pipeline for ROS. This pipeline currently includes scripts to detect shapes of different color in real time with the Kinect v2.

## Dependencies:
1. [Kinect2] (https://github.com/code-iai/iai_kinect2)
2. OpenCV
3. ROS Kinetic

## To run:
1. `roslaunch kinect2_bridge kinect2_bridge.launch`
2. `rosrun vision_pipeline ShapeLabeler.py`

## Tools:
Currently this package has two additional tools for debugging and parameter tuning. in /tools/ there is `hsvfinder.py` which takes in an image as an argument using the `-i` flag. From there you can annotate the image to get maximum and minimum color values in HSV to determine color bands. The second tool is `/tools/prototype_static_detector.py`. As it's name indicates this is useful for testing static images instead of live video.
- use `pcl_install.sh` to install Point Cloud Library from source. I have not been able to find any prebuilt binaries for ubuntu 16.04 so buckle up since it will take 30+ minutes to run
