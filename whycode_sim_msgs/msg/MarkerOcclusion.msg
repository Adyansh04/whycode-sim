# Line-of-sight test from the camera to each marker centre, one message per camera frame.
#
# Only the simulator can answer this: it needs the scene's collision geometry. Published
# separately from MarkerGroundTruthArray so the ground-truth node stays in the ROS
# container and can be re-run against a recorded bag.
#
# header.stamp matches the camera frame exactly, which is what lets the ground-truth node
# join this with the camera pose without interpolation.

std_msgs/Header header

int32[] marker_ids           # parallel to the arrays below, in scene config order
bool[] occluded              # true when geometry blocks the camera-to-marker ray
float32[] hit_distance_m     # distance to the blocking hit, or the marker range if clear
