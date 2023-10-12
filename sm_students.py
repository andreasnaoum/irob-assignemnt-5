#!/usr/bin/env python3

"""

Authors: Andreas Naoum, Adele Robaldo 
Emails: anaoum@kth.se


"""

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from enum import Enum

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


"""
Implement a state machine which goes through the following main states:

    Tuck Arm
    Complete picking task
    Carry cube to second table
    Complete placing task

"""
from enum import Enum
class State(Enum):
    START = 0
    PICK = 1
    CARRY = 2
    PLACE = 3
    FINISH = 4
    ERROR = 5


class StateMachine(object):


    class_name = "State Machine: "


    def __init__(self):
        
        self.node_name = "Student SM"

        rospy.loginfo(self.class_name + "Initialized!")

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')
        self.aruco_pose_tp = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        rospy.loginfo(self.class_name + "Accessed ROS parameters")

        # Wait for service providers
        rospy.wait_for_service(self.pick_srv, timeout=30)
        rospy.wait_for_service(self.place_srv, timeout=30)
        rospy.loginfo(self.class_name + "Wait for service providers finished")

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pos_pub = rospy.Publisher(self.aruco_pose_tp, PoseStamped, queue_size=10)
        rospy.loginfo(self.class_name + "Publishers Initialized")
        
        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = State.START
        rospy.sleep(3)
        self.check_states()

   
    """

    rosmsg show geometry_msgs/PoseStamped
    
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        geometry_msgs/Pose pose
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
    
    """
     # 0.50306828716, 0.0245718046511, 0.915538062216, 0.0144467629456, 0.706141958739, 0.707257659069, -0.0306827123383
    def get_pick_pose(self):
        pick_pose = PoseStamped()
        pick_pose.header.stamp = rospy.Time.now()
        pick_pose.header.frame_id = self.robot_base_frame
        pick_pose.pose.position.x = 0.50306828716 + 0.02
        pick_pose.pose.position.y = 0.0245718046511 + 0.02
        pick_pose.pose.position.z = 0.915538062216 - 0.03
        pick_pose.pose.orientation.x = 0.0144467629456
        pick_pose.pose.orientation.y = 0.706141958739
        pick_pose.pose.orientation.z = 0.707257659069
        pick_pose.pose.orientation.w = -0.0306827123383
        return pick_pose

    def tuck_arm(self):
        rospy.loginfo(self.class_name + "Tucking the arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = True
        self.play_motion_ac.send_goal(goal)
        success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
        if success_tucking:
            rospy.loginfo(self.class_name +"Arm tucked succesfully!")
            self.state = State.PICK
        else:
            self.play_motion_ac.cancel_goal()
            rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
            self.state = State.ERROR
        rospy.sleep(1)


    """
    Pick the cube by publishing a PoseStamped message and calling the pick service

    Note: 
    In manipulation_client.py: self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)
    Pick Service is defined in manipulation_client.py
    """
    def pick(self):
        rospy.loginfo(self.class_name + "Picking cube...")
        try: 
            pick_pose = self.get_pick_pose()
            rospy.loginfo(self.class_name + "Publishing pose..")
            self.aruco_pos_pub.publish(pick_pose)
            rospy.sleep(1)
            pick_client = rospy.ServiceProxy(self.pick_srv, SetBool)
            res = pick_client()
            rospy.loginfo("State Machine: pick request %s", res)
            if res.success == True:
                self.state = State.CARRY
                rospy.loginfo(self.class_name + "Picked the cube successfully")
            else:
                self.state = State.ERROR
                rospy.logerr("Node %s did not pick the cube", self.node_name)

        except rospy.ServiceException as e:
            rospy.logerr("Service call to pick server failed: %s", e)


    """
    Rotate and Go Forwad
    """
    def carry(self):
        rospy.loginfo(self.class_name + "Moving to Table 2...")
        move_msg = Twist()
        move_msg.angular.z = -0.5
        rate = rospy.Rate(10)
        cnt = 0
        rospy.loginfo(self.class_name + "Rotating...")
        while not rospy.is_shutdown() and cnt < 62:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1
        move_msg.linear.x = 0.5
        move_msg.angular.z = 0
        cnt = 0
        rospy.loginfo(self.class_name + "Moving Forward...")
        while not rospy.is_shutdown() and cnt < 17:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1
        self.state = State.PLACE
        rospy.sleep(1)


    """
    Place the cube by calling the place service

    Note: 
    Place Service is defined in manipulation_client.py
    """
    def place(self):
        rospy.loginfo(self.class_name + "Placing the cube to Table 2...")
        rate = rospy.Rate(10)
        try: 
            place_client = rospy.ServiceProxy(self.place_srv, SetBool)        
            res = place_client()
            rospy.loginfo("State Machine: place request %s", res)
            if res.success == True:
                self.state = State.FINISH
                rospy.loginfo(self.class_name + "Placed the cube successfully")
            else:
                self.state = State.ERROR
                rospy.loginfo("StateMachine: Node %s did not place the cube", self.node_name)

        except rospy.ServiceException as e:
            rospy.logerr("Service call to pick server failed: %s", e)

    def check_states(self):

        while not rospy.is_shutdown() and self.state != State.ERROR and self.state != State.FINISH:

            # State 1:  Tuck arm 
            if self.state == State.START:
                self.tuck_arm()

            # State 2:  Pick the cube from Table 1
            elif self.state == State.PICK:
                self.pick()

            # State 3:  Move the robot "manually" to table 2
            elif self.state == State.CARRY:
                self.carry()

            # State 4:  Place the cube to Table 2
            elif self.state == State.PLACE:
                self.place()


        if self.state == State.ERROR:
            rospy.logerr("State Machine: %s State machine failed. Check your code and try again!", self.node_name)
            return
        
        rospy.loginfo(self.class_name + "Finished!")
        return

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
