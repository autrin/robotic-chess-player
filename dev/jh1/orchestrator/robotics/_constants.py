UR10E_ARM_NODE_NAME = "ur10e_control"

# could be anything
ARMATURE_CMDR_GRP_NAME = "manipulator"

# maximum time allowed for the robot to plan a requested maneuver
MAXIMUM_PLANNING_TIME_SECS: int = 6

# Gripper joint name
HAND_E_GRIPPER_JOINT_NAME = "hande_left_finger_joint"

# Gripper open/close positions in meters (finger joint displacement)
# These values can be found in catkin_ws/robotiq/robotiq_description/urdf/robotiq_hande.urdf.xacro
# Traverse the XML hierarchy to:
# robot
#   └── xacro:macro name="robotiq_hande"
#       └── joint name="${prefix}robotiq_hande_joint_finger"
#           └── limit (upper="0.025", lower="0.0")
GRIPPER_OPEN_POSITION = 0.025  # fully open
GRIPPER_CLOSED_POSITION = 0.0065

# Trajectory timing
GRIPPER_MOVE_DURATION = 1.0  # seconds
GRIPPER_COMMAND_TOPIC = "/hande_gripper_controller/command"
