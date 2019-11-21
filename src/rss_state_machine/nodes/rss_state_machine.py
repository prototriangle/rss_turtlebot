#!/usr/bin/env python

import rospy
import math
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs


class State:
    OFF = 0             # robot is initialising
    ON = 1              # robot has initialised
    ISSUE_ACTION = 2    # started going through the action states
    WAITING_ACTION = 3  # started going through the action states
    FINISHED = 4        # action list has been exhausted


class StateMachine:
    def __init__(self):

        self.action_list = rospy.get_param("~action_list")
        """
        The state list if a list of actions and values associated with those actions
            - move: [1.0, 0.2]
            - arm:  [0.2, 0.1, 0.4]
            - grip: [TODO]
        """
        rospy.logdebug("List of actions:")
        rospy.logdebug(self.action_list)
        self.state_feedback_delay = rospy.get_param("~state_feedback_delay", 5.0)
        self.state_feedback_delay *= 10e9  # convert to nanoseconds
        self.run_rate = rospy.get_param("~run_rate", 1.0)

        self.controller_status_topic = rospy.get_param("~controller_status_topic")
        self.arm_status_topic = rospy.get_param("~arm_status_topic")
        self.gripper_status_topic = rospy.get_param("~gripper_status_topic")

        # Publishers
        self.planner_pub = rospy.Publisher("move_position", geometry_msgs.PointStamped, queue_size=1)
        self.arm_pub = rospy.Publisher("arm_position", geometry_msgs.PointStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher("gripper_position", geometry_msgs.PointStamped, queue_size=1)

        # Subscribers
        self.controller_status_sub = rospy.Subscriber(self.controller_status_topic, std_msgs.Bool, self.controlerStatusCallback)
        self.arm_status_sub = rospy.Subscriber(self.controller_status_topic, std_msgs.Bool, self.armStatusCallback)
        self.gripper_status_sub = rospy.Subscriber(self.controller_status_topic, std_msgs.Bool, self.gripperStatusCallback)

        # Initialise state machine
        self.state = State.OFF
        self.n_action_step = None
        self.action_issue_timestamp = None
        self.controller_status = False
        self.arm_status = False
        self.gripper_status = False
        self.run_timer = rospy.Timer(rospy.Duration(1/self.run_rate), self.loopState)

    def controlerStatusCallback(self, msg):
        self.controller_status = msg.data

    def armStatusCallback(self, msg):
        self.arm_status = msg.data

    def gripperStatusCallback(self, msg):
        self.gripper_status = msg.data

    def loopState(self, event):
        timestamp = event.current_real  # in ns
        rospy.logdebug("state_machine: Calling loopState at %s", str(timestamp))
        if self.state == State.OFF:
            if self.controller_status and self.arm_status and self.gripper_status:
                self.state == State.ON
                rospy.loginfo("state_machine: Going to ON state")
            else:
                rospy.logdebug("state_machine: OFF state; other nodes not initialised")
                
        elif self.state == State.ON:
            self.n_action_step = 0
            rospy.loginfo("state_machine: Going into cycle state")

        elif self.state == State.ISSUE_ACTION:
            action = self.action_list[self.n_action_step]   # get action from list
            action_name = action.keys()[0]
            rospy.loginfo("state_machine: Issuing action", action)
            if action_name == "move":
                point = geometry_msgs.PointStamped()
                point.header.stamp = rospy.Time.now()
                point.point.x = action[action_name][0]
                point.point.y = action[action_name][1]
                self.planner_pub.publish(point)
                rospy.logdebug("state_machine: Published point for planner", point)
            elif action_name == "arm":
                # TODO
                pass
            elif action_name == "gripper":
                # TODO
                pass
            self.action_issue_timestamp = timestamp
            self.state = State.WAITING_ACTION

        elif self.state == State.WAITING_ACTION:
            # first check if enough time has passed
            if (timestamp - self.action_issue_timestamp > self.state_feedback_delay):
                if self.controller_status and self.arm_status and self.gripper_status:
                    rospy.loginfo("state_machine: action succesfully completed")
                    self.n_action_step += 1
                    if self.n_action_step >= len(self.action_list):
                        rospy.loginfo("state_machine: exhausted all actions. Going into FINISHED state.")
                        self.state = State.FINISHED
                    else:
                        self.state = State.ISSUE_ACTION
            else:
                rospy.logdebug("state_machine: Still waiting for action issue delay")
                rospy.logdebug("state_machine: Current time diff is", timestamp - self.action_issue_timestamp)

        elif self.state == State.FINISHED:
            rospy.loginfo("state_machine: EYYYYY MISSION SUCCSFULL")
            rospy.loginfo("state_machine: PS. In FINISHED state now so doing nothing")

        else:
            rospy.logerror("state_machine: Unknown state!! ", self.state)



if __name__ == '__main__':
    try:
        rospy.init_node("rss_state_machine", anonymous=False, log_level=rospy.DEBUG)
        state_machine = StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
