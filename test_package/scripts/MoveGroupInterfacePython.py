#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from std_msgs.msg import String

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest

print("===== Start =====")
# Initialize moveit_commanded and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('MoveGroupInterfacePython', anonymous=True)

# Instantiate a Robot Commander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object
group = moveit_commander.MoveGroupCommander("manipulator_i5")

# DisplayTrajectory publisher
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

print ("===== Waiting for RVIZ =====")
rospy.sleep(10)
print("===== Starting =====")

# Get name of reference frame
print("===== Reference frame : %s =====", group.get_planning_frame())

# Get name of end-effector
print("===== Reference frame : %s =====", group.get_end_effector_link())

# Get list of all the group
print("===== Robot Group :")
print(robot.get_group_names())

# Get robot state
print("===== robot state =====")
print(robot.get_current_state())
print("==========")

# Planning to Joint Goal

# joint_goal = group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = 0
# joint_goal[2] = 0
# joint_goal[3] = 0
# joint_goal[4] = 0
# joint_goal[5] = 0

# group.go(joint_goal, wait=True)

# group.stop()


# Planning to Pose goal
print("==========")
quaternion = tf.transformations.quaternion_from_euler(3.14,0,-1.57)

pose_goal = geometry_msgs.msg.Pose()

pose_goal.position.x = 0.0
pose_goal.position.y = 0.0
pose_goal.position.z = 1.0
pose_goal.orientation.x = 90.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0
# pose_goal.orientation.w = 1.0
group.set_pose_target(pose_goal)

plan = group.go(wait=True)

group.stop()

group.clear_pose_targets()

# group.execute(pose_goal, wait=True)