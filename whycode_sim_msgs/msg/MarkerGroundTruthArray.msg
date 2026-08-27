# Ground truth for every marker in the scene, one message per camera frame.
#
# Every configured marker appears every frame, visible or not, so a missed detection
# can be separated from a marker that was never in view without re-running the sim.
# header.stamp is copied verbatim from the sensor_msgs/Image it corresponds to, and
# header.frame_id matches whycode_vision's ros_interface.parent_frame_id.

std_msgs/Header header
int32 visible_marker_count
geometry_msgs/Pose camera_world_pose   # camera in the world frame at this stamp
MarkerGroundTruth[] markers
