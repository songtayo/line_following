# 시작 명령어
## 1. 터틀봇
### 1-1. ros2 launch turtlebot3_bringup robot.launch.py
### 1-2. ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0" -p image_width:=320 -p image_height:=240 -p pixel_format:="mjpeg2rgb" -p camera_name:="test_camera" -p frame_id:="camera_link"

## 2. VM(Ubuntu)
### 2-1.  ros2 launch bringup total_system.launch.py
