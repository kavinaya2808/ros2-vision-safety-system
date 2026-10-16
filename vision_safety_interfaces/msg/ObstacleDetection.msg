# Result of running the obstacle detector on ONE camera image.
#
# header.stamp is the stamp of the SOURCE IMAGE, not the publish time.
# The safety monitor uses it to judge freshness, so a perception pipeline
# that freezes and keeps republishing old results is still caught.

std_msgs/Header header

bool detected                       # a blob of the target colour passed the size filter
bool in_danger_zone                 # the robot must not move forward

sensor_msgs/RegionOfInterest bbox   # bounding box of the largest blob, in pixels
float32 area_ratio                  # blob pixel area / image area, in [0, 1]

# Estimated free distance between the robot's front and the obstacle's base,
# from the ground-plane model. NaN when unknown (no detection, or the base is
# above the horizon). When bottom_clipped is true the base is out of view and
# gap_m is an UPPER bound.
float32 gap_m
bool bottom_clipped
bool in_path                        # lateral extent overlaps the robot's swept path
bool triggered_by_area              # in zone ONLY because of the large-blob safety net

float32 processing_time_ms
