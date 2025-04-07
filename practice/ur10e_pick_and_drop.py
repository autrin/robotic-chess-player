#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

class UR10eControl:
    def __init__(self):
        # Initialize MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur10e_control', anonymous=True)

        # Initialize the move group for UR10e
        self.arm = moveit_commander.MoveGroupCommander("manipulator")

        # Set planning time & reference frame
        self.arm.set_planning_time(10)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.allow_replanning(True)

    
    
    def move_to_joint_positions(self, joint_positions):
        """Moves the arm to a specific joint configuration."""
        self.arm.set_joint_value_target(joint_positions)
        success = self.arm.go(wait=True)

        if not success:
            rospy.logerr("Failed to reach joint configuration!")
        self.arm.stop()
        self.arm.clear_pose_targets()
        
    def reset_position(self):
        """Moves the robot to a predefined reset position where wrist 3 is parallel to the ground and facing downward."""
        rospy.loginfo("Moving to reset position...")
        reset_joint_positions = [
            0.0,    # shoulder_pan (base rotation)
            -1.57,  # shoulder_lift (move arm forward)
            1.57,   # elbow (keep forearm vertical)
            -1.57,  # wrist_1 (ensure wrist faces downward)
            -1.57,  # wrist_2 (align wrist with forearm)
            0.0     # wrist_3 (keep parallel to the ground)
        ]

        self.move_to_joint_positions(reset_joint_positions)

    def move_wrist_vertical(self, distance=-0.6):
        """Moves the wrist vertically by changing the Z-coordinate while maintaining orientation."""
        rospy.loginfo(f"Moving wrist {'up' if distance>0 else 'down'} by {distance} meters...")
        current_pose = self.arm.get_current_pose().pose

        current_pose.position.z += distance  # Move downward by the given distance

        self.arm.set_pose_target(current_pose)

        success = self.arm.go(wait=True)

        if success:
            rospy.loginfo(f"Wrist moved {'up' if distance>0 else 'down'} by {distance} successfully!")
        else:
            rospy.logerr("Failed to move wrist vertically.")

        self.arm.stop()
        self.arm.clear_pose_targets()

    def move_wrist_to_position(self, x, y):
        """Moves the wrist to the specified (x, y) position while keeping the Z coordinate and wrist orientation unchanged."""
        rospy.loginfo(f"Moving wrist to position: x={x}, y={y}...")


        current_pose = self.arm.get_current_pose().pose

        current_z = current_pose.position.z
        current_orientation = current_pose.orientation

        current_pose.position.x = x
        current_pose.position.y = y
        current_pose.position.z = current_z  # Preserve the Z value

        current_pose.orientation = current_orientation  
        self.arm.set_pose_target(current_pose)

        # Execute the movement
        success = self.arm.go(wait=True)

        if success:
            rospy.loginfo(f"Wrist moved to position: x={x}, y={y} successfully!")
        else:
            rospy.logerr("Failed to move wrist to the specified position.")

        self.arm.stop()
        self.arm.clear_pose_targets()


    def move_piece(self, pick_x, pick_y, drop_x, drop_y, move_distance=0.6):
        """Simulates the pick-and-drop routine by moving the wrist to the pick and drop locations."""

        rospy.loginfo("Starting the pick-and-drop routine...")

        # Move to the pick position (x, y)
        rospy.loginfo(f"Moving to pick position: ({pick_x}, {pick_y})")
        self.move_wrist_to_position(pick_x, pick_y)

        # Move wrist down to simulate picking
        rospy.loginfo("Moving wrist down to simulate picking...")
        self.move_wrist_vertical(-move_distance)  # Move down by move_distance (e.g., -0.6)

        rospy.sleep(1)  # Simulate grabbing (gripper not implemented)

        # Lift wrist after pick
        rospy.loginfo("Lifting wrist after pick...")
        self.move_wrist_vertical(move_distance)  # Lift wrist back by move_distance

        # Move to the drop position (x, y)
        rospy.loginfo(f"Moving to drop position: ({drop_x}, {drop_y})")
        self.move_wrist_to_position(drop_x, drop_y)

        # Move wrist down to simulate dropping
        rospy.loginfo("Moving wrist down to simulate dropping...")
        self.move_wrist_vertical(-move_distance)  # Move down to drop the piece

        rospy.sleep(1)  # Simulate releasing (gripper not implemented)

        # Lift wrist after drop
        rospy.loginfo("Lifting wrist after drop...")
        self.move_wrist_vertical(move_distance)  # Lift wrist back

        rospy.loginfo("Pick-and-drop routine complete.")

if __name__ == "__main__":
    robot = UR10eControl()

    # Move to reset position before starting any pick-and-drop actions
    robot.reset_position()

    # robot.move_wrist_vertically( distance=0.6)

    # robot.move_wrist_to_position(0.5, 0.2)
    # robot.move_wrist_vertical(-0.6)
    # robot.move_wrist_vertical(0.6)

    # robot.move_wrist_to_position(0.2, 0.5)
    
    robot.move_piece(0.5, 0.2, 0.2, 0.5)

# You can call this for different positions
    robot.move_piece(0.6, 0.3, 0.1, 0.4)
    
    rospy.signal_shutdown("Task Complete")
