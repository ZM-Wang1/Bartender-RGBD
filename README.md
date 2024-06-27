<!-- ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true -->
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 launch yolov8_bringup yolov8.launch.py input_image_topic:='/camera/color/image_raw' device:='cpu'
ros2 launch detect_bac start.launch.py
<!-- ros2 topic echo /detect_bac -->