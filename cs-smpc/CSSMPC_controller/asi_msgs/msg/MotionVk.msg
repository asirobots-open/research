# MotionVk members
float64                     velocity                    # +v is aligned with the platform +x-axis
float64                     curvature                   # +K produces cw rotation, -K produces ccw rotation about the platform turn center
float64                     distance_to_stop            # Distance to stopping point in meters
uint8                       direction                   # Desired direction (enum MotionVkDirection)
MotionVkFieldsPresent       fields_present              # flags specifying which MotionVk fields are present
bool                        is_modified                 # specifies that the structure has been modified since the previous transmission

# enum MotionVkDirection
uint8                       MOTION_VK_STOPPED   = 0
uint8                       MOTION_VK_FORWARD   = 1
uint8                       MOTION_VK_REVERSE   = 2
