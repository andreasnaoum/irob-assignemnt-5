#!/usr/bin/env python3

"""

Authors: Andreas Naoum, Adele Robaldo 
Emails: anaoum@kth.se, robaldo@kth.se

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

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import numpy as np

from enum import Enum

import py_trees as pt, py_trees_ros as ptr


# ---------------------------- Supportive Enums ---------------------------- #

class ProjectParameters(Enum):
	# Services 
	PICK_SERVICE = '/pick_srv'
	PLACE_SERVICE = '/place_srv'
	LOCALISATION_SERVICE = '/global_loc_srv'
	CLEAR_COSTMAP_SERVICE = '/clear_costmaps_srv'
	# Topics
	VEL_TOPIC = '/cmd_vel_topic'
	PICK_POSE_TOPIC = '/pick_pose_topic'
	PLACE_POSE_TOPIC = '/place_pose_topic'
	ARUCO_POSE_TOPIC = '/aruco_pose_topic'
	AMCL = '/amcl_estimate'

class HeadDirection(Enum):
	UP = "up"
	DOWN = "down"

class GoalPosition(Enum):
	PICK = "pick"
	PLACE = "place"

class NodeStatus(Enum):
    START = 0
    RUNNING = 1
    SUCCESS = 2
    FAILURE = 3

class StudentLevel(Enum):
    A = 0
    C = 1



# ---------------------------- Behaviour Tree ---------------------------- #

class_name = "Behaviour Tree: "
reset_level = 0

class BehaviourTree(ptr.trees.BehaviourTree):

	level = StudentLevel.A


	def __init__(self):

		rospy.loginfo("Behaviour Tree: Initialising behaviour tree")

		# Class Variables
		self.cubePlacedOnTable = False

		action_node_tuck_arm = TuckArm()

		action_node_lower_head = HeadMove(HeadDirection.DOWN)

		action_node_up_head = HeadMove(HeadDirection.UP)

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

		node_reset = RespawnCube(False)

		selector_is_cube_on_table = pt.composites.Selector(
			name="Check if cube is on table", 
			children=[check_placement, sequence_move_to_initial_place, node_reset]
		)

		if self.level == StudentLevel.A:

			action_node_localisation = Localisation()

			action_node_navigation_pick = Navigation(GoalPosition.PICK)

			sequence_pick_cube = pt.composites.Sequence(
				name="Pick cube", 
				children=[
					action_node_tuck_arm, 
					action_node_lower_head,
					action_node_pick,
					action_node_up_head
				]
			)

			action_node_navigation_place = Navigation(GoalPosition.PLACE)

			action_node_navigation_pick1 = Navigation(GoalPosition.PICK)

			node_respawn_cube = RespawnCube(True)

			selector_is_cube_on_table = pt.composites.Selector(
				name="Check if cube is on table", 
				children=[
					check_placement, 
					node_respawn_cube
				]
			)

			action_node_lower_head1 = HeadMove(HeadDirection.DOWN)

			tree = pt.composites.Sequence(
				name="Pick&Carry&Place Sequence", 
				children=[
					action_node_up_head,
					action_node_tuck_arm,
					action_node_localisation,
					action_node_navigation_pick,
					sequence_pick_cube,
					action_node_navigation_place, 
					action_node_place,
					action_node_lower_head1,
					selector_is_cube_on_table
					]
			)

		else:
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



class RespawnCube(pt.behaviour.Behaviour):
		
	def __init__(self, respawn):
		self.respawn = respawn
		rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
		self.respawn_cube = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		super(RespawnCube, self).__init__("Respawn Cube")
		
	def update(self):
		global reset_level
		rospy.loginfo("Respawn Cube and start again")
		if self.respawn:
			cube = ModelState()
			cube.model_name = "aruco_cube"
			cube.pose.position.x = -1.130530
			cube.pose.position.y = -6.653650
			cube.pose.position.z = 0.86250
			cube.pose.orientation.x = 0
			cube.pose.orientation.y = 0
			cube.pose.orientation.z = 0
			cube.pose.orientation.w = 1
			cube.twist.linear.x = 0
			cube.twist.linear.y = 0
			cube.twist.linear.z = 0
			cube.twist.angular.x = 0
			cube.twist.angular.y = 0
			cube.twist.angular.z = 0
			cube.reference_frame = "map"
			self.respawn_cube(cube)
		reset_level += 1
		return pt.common.Status.SUCCESS
	

class Navigation(pt.behaviour.Behaviour):
	
	def __init__(self, goal):

		self.goal = goal
		self.reset_level = reset_level

		self.received = 0

		self.kidnapped = False

		rospy.loginfo(class_name + "Navigation to Pick is initialized!")

		self.amcl_top = rospy.get_param(rospy.get_name() + ProjectParameters.AMCL.value)

		self.loc_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.LOCALISATION_SERVICE.value)
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + ProjectParameters.VEL_TOPIC.value)
		self.clr_costmap_srv = rospy.get_param(rospy.get_name() + ProjectParameters.CLEAR_COSTMAP_SERVICE.value)
		rospy.wait_for_service(self.loc_srv_nm, timeout=30)
		rospy.wait_for_service(self.clr_costmap_srv, timeout=30)

		self.loc_srv = rospy.ServiceProxy(self.loc_srv_nm, Empty)
		self.clear_costmap_srv = rospy.ServiceProxy(self.clr_costmap_srv, Empty)
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		# self.estimate_pos = rospy.get_param(rospy.get_name() + ProjectParameters.AMCL.value)
		# self.detecting_cube = rospy.Subscriber(self.estimate_pos, PoseWithCovarianceStamped, self.position_msg)

		if self.goal == GoalPosition.PICK:
			# self.pick_topic = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
			self.pick_topic = rospy.get_param(rospy.get_name() + ProjectParameters.PICK_POSE_TOPIC.value)
			self.goal_pose = rospy.wait_for_message(self.pick_topic, PoseStamped, 30)
		else:
			self.place_topic = rospy.get_param(rospy.get_name() + ProjectParameters.PLACE_POSE_TOPIC.value)
			self.goal_pose = rospy.wait_for_message(self.place_topic, PoseStamped, 30)

		self.move_action = SimpleActionClient("/move_base", MoveBaseAction)
		if not self.move_action.wait_for_server(rospy.Duration(1000)):
			rospy.logerr('Cannot connect to move_base')

		rospy.loginfo(class_name + "Navigation connect with move_base")

		self.nav_status = NodeStatus.START

		# become a behaviour
		super(Navigation, self).__init__("Navigation!")


	def update(self):

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.nav_status = NodeStatus.START

		if self.nav_status == NodeStatus.SUCCESS:
			return pt.common.Status.SUCCESS
		elif self.nav_status == NodeStatus.FAILURE:
			return pt.common.Status.FAILURE
		elif self.nav_status == NodeStatus.START:
			# rospy.sleep(2)
			print('Navigation called' + str(self.nav_status))
			self.nav_status = NodeStatus.RUNNING
			self.move_goal = MoveBaseGoal()
			self.move_goal.target_pose = self.goal_pose
			# feedback_cb=self.goal_check, 
			self.move_action.send_goal(self.move_goal, feedback_cb=self.goal_check, done_cb=self.goal_finish)
			self.nav_result = self.move_action.wait_for_result(rospy.Duration(60.0))
			return pt.common.Status.RUNNING
		else:
			return pt.common.Status.RUNNING
		
	def goal_finish(self, state, result):

		if self.kidnapped:
			print("Goal finish return")
			return
		if result:
			self.nav_status = NodeStatus.SUCCESS
			print('Success: Navigation ' + str(self.nav_status))
		else:
			self.nav_status = NodeStatus.FAILURE
			print('Failure: Navigation ' + str(self.nav_status))

	def goal_check(self, feedback):
		if self.nav_status == NodeStatus.RUNNING and feedback:
			self.received += 1
			print(class_name + " Moving: " + str(self.received))
			particle = rospy.wait_for_message(self.amcl_top, PoseWithCovarianceStamped, 5)
			cov = np.linalg.norm(particle.pose.covariance)
			# print("Covariance: " + str(cov))
			if self.kidnapped == False and (cov > 0.05) and self.received > 10:
				print("Robot is KIDNAPPED!" + str(cov))
				self.kidnapped = True
				self.move_action.cancel_goal()

				self.loc_srv()

				rospy.loginfo(class_name + 'Robot is localising. Status: '+ str(NodeStatus.SUCCESS))

				rate = rospy.Rate(10)
				self.cntt = 0

				self.move_msg = Twist()
				self.move_msg.angular.z = -2 

				while not rospy.is_shutdown() and self.cntt<60:
					self.cmd_vel_pub.publish(self.move_msg)
					rate.sleep()
					self.cntt = self.cntt + 1

				self.clear_costmap_req = self.clear_costmap_srv()

				self.move_goal = MoveBaseGoal()
				self.move_goal.target_pose = self.goal_pose
				self.kidnapped = False
				self.move_action.send_goal(self.move_goal, feedback_cb=self.goal_check, done_cb=self.goal_finish)
				self.nav_result = self.move_action.wait_for_result(rospy.Duration(100.0))

				
				



		# print("Feedback " + str(feedback))


class Localisation(pt.behaviour.Behaviour):

	def __init__(self):
		global reset_level

		self.reset_level = reset_level

		rospy.loginfo(class_name + "Localisation is initialized!")

		self.loc_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.LOCALISATION_SERVICE.value)
		rospy.wait_for_service(self.loc_srv_nm, timeout=30)
		self.loc_srv = rospy.ServiceProxy(self.loc_srv_nm, Empty)
		
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + ProjectParameters.VEL_TOPIC.value)
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		
		self.clr_costmap_srv = rospy.get_param(rospy.get_name() + ProjectParameters.CLEAR_COSTMAP_SERVICE.value)
		rospy.wait_for_service(self.clr_costmap_srv, timeout=30)
		self.clear_costmap_srv = rospy.ServiceProxy(self.clr_costmap_srv, Empty)

		self.loc_status = NodeStatus.START
		self.cntt = 0

		# become a behaviour
		super(Localisation, self).__init__("Localisation!")


	def update(self):
		global reset_level

		if self.loc_status == NodeStatus.SUCCESS:
			return pt.common.Status.SUCCESS
		elif self.loc_status == NodeStatus.FAILURE:
			return pt.common.Status.FAILURE
		elif self.loc_status == NodeStatus.START:
			self.local_req = self.loc_srv()
			self.loc_status = NodeStatus.RUNNING
			self.move_msg = Twist()
			self.move_msg.angular.z = -2 

			rospy.loginfo(class_name + 'Robot is localising. Status: '+ str(NodeStatus.SUCCESS))

			rate = rospy.Rate(10)
			self.cntt = 0

			while not rospy.is_shutdown() and self.cntt<60:
				self.cmd_vel_pub.publish(self.move_msg)
				rate.sleep()
				self.cntt = self.cntt + 1
				# print('Counter: ' + str(self.cntt))

			self.clear_costmap_req = self.clear_costmap_srv()
			self.loc_status = NodeStatus.SUCCESS
			print('Localisation Success: ')

			self.cntt = 0

			while not rospy.is_shutdown() and self.cntt<15:
				rate.sleep()
				self.cntt = self.cntt + 1


			return pt.common.Status.SUCCESS			
		else:
			return pt.common.Status.RUNNING
				

class CheckPlacement(pt.behaviour.Behaviour):

	def __init__(self):
		global reset_level

		self.reset_level = reset_level

		rospy.loginfo(class_name + "Check Placement is initialized!")

		self.test_scenario_no_cube_on_table = False
		self.wait_threshold = 100
		self.timer = 0
		
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + ProjectParameters.ARUCO_POSE_TOPIC.value)

		self.check_status = NodeStatus.START

		self.aruco_pose_rcv = False

		# become a behaviour
		super(CheckPlacement, self).__init__("Check Cube Placement!")


	def update(self):
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.check_status = NodeStatus.START
			return pt.common.Status.RUNNING

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

			# rospy.sleep(1)
			
			if self.aruco_pose_rcv:
				rospy.loginfo(class_name + "Cube is on the table!")
				self.check_status = NodeStatus.SUCCESS
				return pt.common.Status.SUCCESS
			elif self.timer > self.wait_threshold:
				self.check_status = NodeStatus.FAILURE
				print(class_name + "Check Placement Returns Failure")
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
		global reset_level

		self.reset_level = reset_level

		rospy.loginfo(class_name + "Pick Action Node is initialised!")
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.PICK_SERVICE.value)
		rospy.wait_for_service(self.pick_srv_nm, timeout=30)
		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

		self.pick_status = NodeStatus.START

		# become a behaviour
		super(Pick, self).__init__("Pick Cube")

	def update(self):
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.pick_status = NodeStatus.START

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
		global reset_level

		self.reset_level = reset_level

		rospy.loginfo(class_name + "Place Action Node is initialised!")

		self.place_srv_nm = rospy.get_param(rospy.get_name() + ProjectParameters.PLACE_SERVICE.value)
		rospy.wait_for_service(self.place_srv_nm, timeout= 30)
		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

		self.place_status = NodeStatus.START

		# become a behaviour
		super(Place, self).__init__("Place Cube")

	def update(self):
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.place_status = NodeStatus.START

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
		global reset_level

		self.reset_level = reset_level

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
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.tried = False

		# try to tuck head if haven't already
		if not self.tried:

			# rospy.sleep(5.0)
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
		global reset_level

		self.reset_level = reset_level

		rospy.loginfo(class_name + "Counter is called!")

		# counter
		self.i = 0
		self.n = n

		# become a behaviour
		super(Counter, self).__init__(name)

	def update(self):
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.i = 0

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
		global reset_level

		self.reset_level = reset_level

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
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.sent_goal = False
			self.finished = False

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
		global reset_level

		self.reset_level = reset_level

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
		global reset_level

		if self.reset_level < reset_level:
			self.reset_level = reset_level
			self.tried = False

		# try to tuck head if haven't already
		if not self.tried:

			# command
			self.move_head_req = self.move_head_srv("down")
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# react to outcome
		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE

		
	def position_msg(self, pose_msg):
		self.robot_pose = pose_msg
		self.update_pose = True


# ---------------------------- Main ---------------------------- #

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
