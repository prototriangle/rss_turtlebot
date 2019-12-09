#!/usr/bin/env python

import rospy
import math
from enum import Enum
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
from rss_ik.msg import SendIKCommand


class State(Enum):
    OFF = 0             # robot is initialising
    ON = 1              # robot has initialised
    ISSUE_ACTION = 2    # started going through the action states
    WAITING_MOVE = 3    # started going through the action states
    WAITING_ARM = 4
    WAITING_GRIPPER = 5
    FINISHED = 6          # action list has been exhausted


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
        self.run_rate = rospy.get_param("~run_rate", 1.0)

        self.controller_status_topic = rospy.get_param("~controller_status_topic")
        self.arm_status_topic = rospy.get_param("~arm_status_topic")
        self.gripper_status_topic = rospy.get_param("~gripper_status_topic")

        # Publishers
        self.planner_pub = rospy.Publisher("~move_position", geometry_msgs.PointStamped, queue_size=1)
        self.arm_pub = rospy.Publisher("~arm_position", SendIKCommand, queue_size=1)
        self.gripper_pub = rospy.Publisher("~gripper_position", geometry_msgs.PointStamped, queue_size=1)

        # Subscribers
        self.controller_status_sub = rospy.Subscriber(
            self.controller_status_topic, std_msgs.Int16, self.controlerStatusCallback)
        self.arm_status_sub = rospy.Subscriber(self.controller_status_topic, std_msgs.Int16, self.armStatusCallback)
        self.gripper_status_sub = rospy.Subscriber(
            self.controller_status_topic, std_msgs.Int16, self.gripperStatusCallback)

        # Initialise state machine
        self.state = State.OFF
        self.n_action_step = None
        self.action_issue_timestamp = None
        self.controller_status = 999
        self.arm_status = 999
        self.gripper_status = 999

        # This is the thing that calls the state transition loop
        self.run_timer = rospy.Timer(rospy.Duration(1/self.run_rate), self.loopState)

    def controlerStatusCallback(self, msg):
        self.controller_status = msg.data

    def armStatusCallback(self, msg):
        self.arm_status = msg.data

    def gripperStatusCallback(self, msg):
        self.gripper_status = msg.data

    def loopState(self, event):
        """This is the core of the state machine. Does all of the
        statte transitions"""
        timestamp = event.current_real  # in ns
        rospy.logdebug("state_machine: Calling loopState at %s", str(timestamp))
        rospy.logdebug("state_machine: Current state is %s", self.state.name)

        # Start switch statement
        if self.state == State.OFF:
            if (self.controller_status == 0) and (self.arm_status == 0):
                rospy.loginfo("state_machine: Going to ON state")
                self.state = State.ON
                self.action_issue_timestamp = timestamp
            else:
                rospy.logdebug("state_machine: OFF state; other nodes not initialised")

        elif self.state == State.ON:
            if (timestamp - self.action_issue_timestamp) > rospy.Duration.from_sec(10):
                self.n_action_step = 0
                rospy.loginfo("state_machine: Going into ISSUE_ACTION state")
                self.state = State.ISSUE_ACTION

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
                rospy.logdebug("state_machine: Published point for planner")
                self.state = State.WAITING_MOVE
            elif action_name == "arm":
                s = SendIKCommand()
                s.x = action[action_name][0]
                s.y = action[action_name][1]
                s.command = action[action_name][2]
                self.arm_pub.publish(s)
                rospy.logdebug("state_machine: Published command for arm")
                self.state = State.WAITING_ARM
                pass
            else:
                rospy.logfatal("state_machine: action name not recognised: ", action_name)
            self.action_issue_timestamp = timestamp

        elif self.state == State.WAITING_MOVE:
            # first check if enough time has passed
            if (timestamp - self.action_issue_timestamp) > rospy.Duration.from_sec(self.state_feedback_delay):
                if self.controller_status == 2:
                    rospy.loginfo("state_machine: action succesfully completed")
                    self.n_action_step += 1
                    if self.n_action_step >= len(self.action_list):
                        rospy.loginfo("state_machine: exhausted all actions. Going into FINISHED state.")
                        self.state = State.FINISHED
                    else:
                        self.state = State.ISSUE_ACTION
            else:
                rospy.logdebug("state_machine: Still waiting for action issue delay")

        elif self.state == State.WAITING_ARM:
            # first check if enough time has passed
            if (timestamp - self.action_issue_timestamp) > rospy.Duration.from_sec(self.state_feedback_delay):
                rospy.loginfo("state_machine: arm_status=%d", self.arm_status)
                if self.arm_status == 2:
                    rospy.loginfo("state_machine: action succesfully completed")
                    self.n_action_step += 1
                    if self.n_action_step >= len(self.action_list):
                        rospy.loginfo("state_machine: exhausted all actions. Going into FINISHED state.")
                        self.state = State.FINISHED
                    else:
                        self.state = State.ISSUE_ACTION
            else:
                rospy.logdebug("state_machine: Still waiting for action issue delay")

        elif self.state == State.WAITING_MOVE:
            # first check if enough time has passed
            if (timestamp - self.action_issue_timestamp > self.state_feedback_delay):
                if self.gripper_status == 2:
                    rospy.loginfo("state_machine: action succesfully completed")
                    self.n_action_step += 1
                    if self.n_action_step >= len(self.action_list):
                        rospy.loginfo("state_machine: exhausted all actions. Going into FINISHED state.")
                        self.state = State.FINISHED
                    else:
                        self.state = State.ISSUE_ACTION
            else:
                rospy.logdebug("state_machine: Still waiting for action issue delay")

        elif self.state == State.FINISHED:
            rospy.loginfo("state_machine: EYYYYY MISSION FINISHED! WOOP WOOP!")
            rospy.loginfo("state_machine: PS. In FINISHED state now so doing nothing")

        else:
            rospy.logerror("state_machine: Unknown state!! ", self.state)


if __name__ == '__main__':
    try:
        # rospy.init_node("rss_state_machine", anonymous=False, log_level=rospy.DEBUG)
        rospy.init_node("rss_state_machine", anonymous=False)
        state_machine = StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
