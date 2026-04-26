# A single detected object from the perception pipeline

string object_id                   # Unique ID for this detection instance
string class_name                  # Object class label, e.g. "seed", "cup"
float64 confidence                 # Detection confidence 0.0-1.0
geometry_msgs/PoseStamped pose     # 3D pose in world/camera frame
sensor_msgs/RegionOfInterest bbox_2d  # 2D bounding box in image
float32 width                      # Estimated object width in meters
float32 height                     # Estimated object height in meters
float32 depth                      # Estimated object depth in meters
string[] attributes                # Additional attributes, e.g. ["color:red", "shape:sphere"]
