# Exact pose of one simulated fiducial marker relative to the camera.
#
# CONVENTION: `pose` uses the same frame as whycode_vision/msg/WhyCodePose --
# camera body frame, X forward along the optical axis, Y left, Z up (REP-103).
# This is NOT the camera optical frame. The detector permutes the optical triple
# in whycon_localization.cpp:610-613 before publishing, so these two topics can be
# differenced field by field with no transform.

int32 marker_id                 # decoded ID (necklace code / AprilTag id), NOT the file index
string family                   # "whycode" or "apriltag"
string texture                  # texture basename applied, e.g. "whycode_3"

geometry_msgs/Pose pose         # marker in the camera frame; diff against WhyCodePose.pose
geometry_msgs/Pose world_pose   # marker in the world frame

float64 range_m                 # camera origin to marker centre
float64 incidence_angle_deg     # marker normal vs marker-to-camera ray, 0 = head-on
float64 bearing_u_px            # pinhole projection of the centre, may fall outside the image
float64 bearing_v_px
float64 apparent_diameter_px    # size_m * fx * cos(incidence) / range_m

bool in_frustum                 # all four corners project inside the image, past the near plane
bool facing_camera              # incidence_angle_deg < visibility.max_incidence_deg
bool large_enough               # apparent_diameter_px >= visibility.min_diameter_px
bool occluded                   # scene raycast from camera to marker centre hit something first
bool visible                    # in_frustum and facing_camera and large_enough and not occluded
