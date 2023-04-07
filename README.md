# orb_slam_ros

ROS2 wrapper for ORB_SLAM3

To get current map_id from ORB_SLAM3, 
use [my fork](https://github.com/knb/ORB_SLAM3 "knb/ORB_SLAM3") ORB_SLAM3.

You may need to change CmakeList.txt.

````
  # set ORB_SLAM3 directory
  set(ORB_SLAM3_PATH ~/src/ORB_SLAM3)
````

## params

for publish tf map to odom.

- map_frame_id  - "map"
- target_frame_id - "odom"
- camera_frame_id - "camera_link"

for ORB_SLAM3 (stereo)
(monocular is not work yet)

- voc_file - set full path for voc_file
- settings_file - set full path for param yaml file.
- use_viewer - display viewer if true.
- publish_raw - no tf publish. publish only raw Twc topic.
- image_left_topic - "/camera/image_left_raw"
- image_right_topic - "/camera/image_right_raw"
