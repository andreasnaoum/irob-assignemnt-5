#!/usr/bin/env python3

"""

Authors: Andreas Naoum, Adele Robaldo 
Emails: anaoum@kth.se

Launch Simulator: roslaunch robotics_project gazebo_project.launch
Start: roslaunch robotics_project launch_project.launch


"""

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

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

import py_trees as pt, py_trees_ros as ptr


# ---------------------------- Supportive Enums ---------------------------- #

class ProjectParameters(Enum):
	PICK_SERVICE = '/pick_srv'
	PLACE_SERVICE = '/place_srv'
	ARUCO_POSE_TOPIC = '/aruco_pose_topic'


class HeadDirection(Enum):
	UP = "up"
	DOWN = "down"


class NodeStatus(Enum):
    START = 0
    RUNNING = 1
    SUCCESS = 2
    FAILURE = 3

# ---------------------------- Behaviour Tree ---------------------------- #

class_name = "Behaviour Tree: "

class BehaviourTree(ptr.trees.BehaviourTree):


	def __init__(self):

		rospy.loginfo("Behaviour Tree: Initialising behaviour tree")

		# Class Variables
		self.cubePlacedOnTable = False

		action_node_tuck_arm = TuckArm()

		action_node_lower_head = HeadMove(HeadDirection.DOWN)

		# action_node_up_head = HeadMove(HeadDirection.UP)

		action_node_pick = Pick()

		selector_rotate = pt.composites.Selector(
			name="Rotate", 
			children=[Counter(60, "Check Rotate Finish"), Go("Rotating", 0, -0.5)]
		)

		selector_go_forward = pt.composites.Selector(
			name="Go Forward", 
			children=[Counter(17, "Check Forward Finish"), Go("Moving", 0.5, 0)]
		)

		sequence_carry = pt.composites.Sequence(
			name="Move to Table Two Sequence", 
			children=[selector_rotate, selector_go_forward]
		)

		action_node_place = Place()

		check_placement = CheckPlacement()

		selector_rotate1 = pt.composites.Selector(
			name="Rotate", 
			children=[Counter(60, "Check Rotate Finish"), Go("rotating", 0, -0.5)]
		)

		selector_go_forward1 = pt.composites.Selector(
			name="Go Forward", 
			children=[Counter(17, "Check Forward Finish"), Go("moving forward", 0.5, 0)]
		)

		sequence_move_to_initial_place = pt.composites.Sequence(
			name="Move to Table One Sequence", 
			children=[selector_rotate1, selector_go_forward1]
		)

		selector_is_cube_on_table = pt.composites.Selector(
			name="Check if cube is on table", 
			children=[check_placement, sequence_move_to_initial_place]
		)

		tree = pt.composites.Sequence(
			name="Pick&Carry&Place Sequence", 
			children=[
				action_node_tuck_arm, 
				action_node_lower_head, 
				action_node_pick,
				sequence_carry, 
				action_node_place,
				selector_is_cube_on_table
				]
		)

		rospy.sleep(2)
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)


# ---------------------------- Tree Nodes ---------------------------- #

class CheckPlacement(pt.behaviour.Behaviour):

	

	def __init__(self):

		rospy.loginfo(class_name + "Check Placement is initialized!")

		self.test_scenario_no_cube_on_table = False
		self.wait_threshold = 50
		
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + ProjectParameters.ARUCO_POSE_TOPIC.value)

		self.check_status = NodeStatus.START

		self.aruco_pose_rcv = False

		self.timer = 0

		# become a behaviour
		super(CheckPlacement, self).__init__("Check Cube Placement!")


	def update(self):

		if self.check_status == NodeStatus.SUCCESS:
			return pt.common.Status.SUCCESS
		elif self.check_status == NodeStatus.FAILURE:
			return pt.common.Status.FAILURE
		elif self.check_status == NodeStatus.START:
			rospy.loginfo(class_name + "Check Placement starts")
			self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)
			self.check_status = NodeStatus.RUNNING
			return pt.common.Status.RUNNING
		else:
			
			if self.aruco_pose_rcv:
				rospy.loginfo(class_name + "Cube is on the table!")
				self.check_status = NodeStatus.SUCCESS
				return pt.common.Status.SUCCESS
			elif self.timer > self.wait_threshold:
				self.check_status = NodeStatus.FAILURE
				print("Returns Failure")
				return pt.common.Status.FAILURE
			else:
				rospy.loginfo(class_name + "Check Placement Running Status ... " + str(self.timer) + "/" + str(self.wait_threshold))
				self.timer += 1
				return pt.common.Status.RUNNING
	
	def aruco_pose_cb(self, aruco_pose_msg):
		self.aruco_pose = aruco_pose_msg
		if not self.test_scenario_no_cube_on_table:
			if not self.aruco_pose_rcv:
				rospy.loginfo(class_name + "Aruco pose msg received!")
				self.aruco_pose_rcv = True

# Action Node - Pick
class Pick(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo(class_name + "Pick Action Node is initialised!")
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.PICK_SERVICE.value)
		rospy.wait_for_service(self.pick_srv_nm, timeout=30)
		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

		self.pick_status = NodeStatus.START

		# become a behaviour
		super(Pick, self).__init__("Pick Cube")

	def update(self):

		if self.pick_status == NodeStatus.SUCCESS:
			return pt.common.Status.SUCCESS
		
		elif self.pick_status == NodeStatus.FAILURE:
			return pt.common.Status.FAILURE
		
		elif self.pick_status == NodeStatus.START:
			rospy.loginfo(class_name + "Pick starts")
			try: 
				self.pick_req = self.pick_srv()
				rospy.loginfo(class_name + "Pick service is called!")
			except rospy.ServiceException as e:
				rospy.logerr(class_name + "Service call to pick server failed with error " + e)
			
			self.pick_status = NodeStatus.RUNNING
			return pt.common.Status.RUNNING
		
		elif self.pick_status == NodeStatus.RUNNING:

			if self.pick_req.success:
				self.pick_status = NodeStatus.SUCCESS
				rospy.loginfo(class_name + "Pick succeed!")
				return pt.common.Status.SUCCESS
		
			elif not self.pick_req.success:
				self.pick_status = NodeStatus.FAILURE
				rospy.loginfo(class_name + "Pick failed!")
				return pt.common.Status.FAILURE
			
			else:
				return pt.common.Status.RUNNING
		else:
			print("Pick else")
			return pt.common.Status.RUNNING

# Action Node - Place
class Place(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo(class_name + "Place Action Node is initialised!")

		self.place_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.PLACE_SERVICE.value)
		rospy.wait_for_service(self.place_srv_nm, timeout= 30)
		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

		self.place_status = NodeStatus.START

		# become a behaviour
		super(Place, self).__init__("Place Cube")

	def update(self):

		if self.place_status == NodeStatus.SUCCESS:
			return pt.common.Status.SUCCESS
		
		elif self.place_status == NodeStatus.FAILURE:
			return pt.common.Status.FAILURE
		
		elif self.place_status == NodeStatus.START:
			rospy.loginfo(class_name + "Place starts")
			try: 
				self.place_req = self.place_srv()
				rospy.loginfo(class_name + "Place service is called!")
			except rospy.ServiceException as e:
				rospy.logerr(class_name + "Service call to place server failed with error " + e)
			
			self.place_status = NodeStatus.RUNNING
			return pt.common.Status.RUNNING
		
		elif self.place_status == NodeStatus.RUNNING:

			if self.place_req.success:
				self.place_status = NodeStatus.SUCCESS
				rospy.loginfo(class_name + "Place succeed!")
				return pt.common.Status.SUCCESS
		
			elif not self.place_req.success:
				self.place_status = NodeStatus.FAILURE
				rospy.loginfo(class_name + "Place failed!")
				return pt.common.Status.FAILURE
			
			else:
				return pt.common.Status.RUNNING
		else:
			print("Place else")
			return pt.common.Status.RUNNING


# Action Node - Move Head 
# Takes "up" or "down" and move the head accordingly
class HeadMove(pt.behaviour.Behaviour):

	def __init__(self, head_direction):

		rospy.loginfo(class_name + "Head Move is called with direction " + head_direction.value + "!")

		self.direction = head_direction

		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

		# execution checker
		self.tried = False
		self.tucked = False

		# become a behaviour
		super(HeadMove, self).__init__("Move head!")

	def update(self):

		# try to tuck head if haven't already
		if not self.tried:

			# command
			self.move_head_req = self.move_head_srv(self.direction.value)
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# react to outcome
		elif self.move_head_req.success:
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.FAILURE	


# ---------------------------- Provided Tree Nodes ---------------------------- #

# Condition Node - Counter
class Counter(pt.behaviour.Behaviour):

	def __init__(self, n, name):

		rospy.loginfo(class_name + "Counter is called!")

		# counter
		self.i = 0
		self.n = n

		# become a behaviour
		super(Counter, self).__init__(name)

	def update(self):

		# count until n
		while self.i <= self.n:

			# increment count
			self.i += 1

			# return failure :(
			return pt.common.Status.FAILURE

		# succeed after counter done :)
		return pt.common.Status.SUCCESS


# Action Node - Go
# Takes name, linear and angular and moves the robot accordingly
class Go(pt.behaviour.Behaviour):

	def __init__(self, name, linear, angular):

		rospy.loginfo(class_name + "Go is called!")

		# action space
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		self.name = name

		# command
		self.move_msg = Twist()
		self.move_msg.linear.x = linear
		self.move_msg.angular.z = angular

		# become a behaviour
		super(Go, self).__init__(name)

	def update(self):

		rospy.loginfo(class_name + "Robot is "+ str(self.name))

		# send the message
		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()

		# tell the tree that you're running
		return pt.common.Status.RUNNING


# Action Node - Tuck Arm
class TuckArm(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo(class_name + "Tuck Arm is called!")

		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True

		# execution checker
		self.sent_goal = False
		self.finished = False

		# become a behaviour
		super(TuckArm, self).__init__("Tuck arm!")

	def update(self):

		# already tucked the arm
		if self.finished: 
			return pt.common.Status.SUCCESS
		
		# command to tuck arm if haven't already
		elif not self.sent_goal:

			# send the goal
			self.play_motion_ac.send_goal(self.goal)
			self.sent_goal = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if I was succesful! :)))))))))
		elif self.play_motion_ac.get_result():

			# than I'm finished!
			self.finished = True
			return pt.common.Status.SUCCESS

		# if I'm still trying :|
		else:
			return pt.common.Status.RUNNING
		

# Action Node - Lower Head 
# Given action node for lower head
class LowerHead(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo("Behaviour Tree: Lower Head is called!")

		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

		# execution checker
		self.tried = False
		self.tucked = False

		# become a behaviour
		super(LowerHead, self).__init__("Lower head!")

	def update(self):

		# try to tuck head if haven't already
		if not self.tried:

			# command
			self.move_head_req = self.move_head_srv("down")
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# react to outcome
		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


# ---------------------------- Main ---------------------------- #

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
