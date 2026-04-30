"""seed_detector — YOLO-based seed detection on a RealSense camera.

Subscribes to RGB + aligned depth + camera_info; runs ultralytics YOLO on
the RGB frame; deprojects each detection's bbox center to 3D via the
camera intrinsics; publishes vision_msgs/Detection3DArray in the camera
optical frame. With debug_markers=true also publishes a
visualization_msgs/MarkerArray for rviz.
"""
