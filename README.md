# foxglove_compressed_video_stream
Stream foxglvoe compressed video messgaes from a /Image message topics from an arbritray camera. Developed with accelerated Gstreamer on Jetson Orin. Example developed with the OAK-D PoE. 

## Dependancies
- DepthAI ROS Driver
- Foxglove Studio + Foxglove Bridge
- (Accelerated) Gstreamer
- OpenCV
- ROS2 cv_bridge

## Installation
### Step 1 - Clone the Repository

Clone the repository into your workspace folder. 

```bash
    git clone https://github.com/HJGrant/foxglove_compressed_video_stream.git
```

### Step 2 - Install Dependancies

```bash
    rosdep update && rosdep install --from-paths src/
```

### Step 3 - Build the Package

```bash
    colcon build
```

### Step 4 - Run the example launch file

```bash
    ros2 launch foxglove_compressed_video compressed_video.launch.py
```

### Step 5 - View the stream

Open Foxglove Studio and connect to the foxglove_bridge instance. You should be able to view the compressed stream by subscribing to the topic ```/compressed_video```.