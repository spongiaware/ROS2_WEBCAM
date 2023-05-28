# Webcam ROS2 Node

This ROS2 node captures video frames from the webcam and publishes them on two topics: `webcam/image_raw` and `webcam/image_processed`. Additionally, it publishes the relative coordinates of any detected faces on the topic `webcam/webcam_coordinates`.

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy Fitzroy (or later)
- Python 3
- OpenCV Python package
- `cv_bridge` package

## Build

1. Create a ROS2 workspace (if you haven't already):


mkdir -p ~/ros2_webcam_ws/src
cd ~/ros2_ws

2. Clone the webcam_pkg package into the src directory:

cd ~/ros2_webcam_ws/src
git clone https://github.com/spongiaware/ROS2_WEBCAM

3. Build the package:

cd ~/ros2_webcam_ws
colcon build

## Run

1. Open a new terminal and source your ROS2 workspace:

source ~/ros2_webcam_ws/install/setup.bash

2. Run the webcam node:

ros2 run webcam_pkg webcam_publisher

## View Image Streams with rqt_image_view

1. Open a new terminal and source your ROS2 workspace:

source ~/ros2_ws/install/setup.bash

2. Launch rqt_image_view:

ros2 run rqt_image_view rqt_image_view
In rqt_image_view, select the desired topic to view the image streams:
- To view the raw webcam image, select the topic: /webcam/image_raw
- To view the processed webcam image, select the topic: /webcam/image_processed