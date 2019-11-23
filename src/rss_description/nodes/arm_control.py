#!/usr/bin/env python

# Note that twistToAckermannDrive needs to be mapped to the ros_can_sim/command parameter in the command line ie. rosrun eufs_ros_can_sim twist_to_ackermannDrive.py twistToAckermannDrive:=eufs_ros_can_sim/command
# Publishes to twistToAckermannDrive
# Subscribes to cmd_vel

import rospy
import math
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


class ArmController:
    def __init__(self):
        self.command_sub = rospy.Subscriber('/joint_trajectory_point', Float64MultiArray, self.callback)
        self.arm_control_publiser = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.grip_control_publiser = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

    def clean_joint_states(self, data):
        # invert joint orientation to match real world
        data = np.array(data)
        data[2] = -data[2]

        lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
        upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
        clean_lower = np.maximum(lower_limits, data)
        clean_upper = np.minimum(clean_lower, upper_limits)
        return list(clean_upper)

    def callback(self, msg):
        #   Joint Position vector should contain 6 elements:
        #   [0, shoulder1, shoulder2, elbow, wrist, gripper]
        if len(msg.data) != 6:
            rospy.WARN("Not enough input positions given. Need to be 6!")
            return

        # Hangle arm control
        arm_cmd_msg = JointTrajectory()
        # arm_cmd_msg.header.stamp = rospy.Time.now()
        arm_cmd_msg.joint_names = ["waist", "shoulder", "elbow", "wrist_angle"]
        new_pt = JointTrajectoryPoint()
        new_pt.time_from_start = rospy.Time(0.1)
        joint_data = self.clean_joint_states(msg.data)
        for pos in joint_data[1:-1]:
            new_pt.positions.append(pos)
        arm_cmd_msg.points.append(new_pt)

        self.arm_control_publiser.publish(arm_cmd_msg)

        # Handle grip control
        grip_cmd_msg = JointTrajectory()
        grip_cmd_msg.header.stamp = rospy.Time.now()
        grip_cmd_msg.joint_names = ["left_finger", "right_finger"]
        new_pt = JointTrajectoryPoint()
        new_pt.time_from_start = rospy.Time(0.1)
        new_pt.positions.append(joint_data[-1])
        new_pt.positions.append(joint_data[-1])
        grip_cmd_msg.points.append(new_pt)
        self.grip_control_publiser.publish(grip_cmd_msg)


if __name__ == '__main__':
    try:
        rospy.init_node("arm_controller", anonymous=False)
        ctrl = ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
