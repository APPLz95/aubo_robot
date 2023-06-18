#!/usr/bin/env python2

# Import namespace
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs

try :
    from math import pi, tau, dist, fabs, cosh
except :
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Initialize Node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_interface", anonymous=True)

# Instantiate RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate PlanningSceneInterface objet
scene = moveit_commander.PlanningSceneInterface()

# Instaniate MoveGroupCommander
group_name = "AUBO_I5"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Create DisplayTrajectory
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Get infermation
planing_frame = move_group.get_planning_frame()
print("Planning frame : %s" % planing_frame)

eef_link = move_group.get_end_effector_link()
print("End effector link : %s" % eef_link)

group_names = robot.get_group_names()
print("Available Planing Group : ", robot.get_group_names())

print("Printing robot state")
print(robot.get_current_state())
print("")

# Planning to Joint Goal
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -tau / 8
joint_goal[2] = 0
joint_goal[3] = -tau / 4
joint_goal[4] = 0
joint_goal[5] = tau / 6
joint_goal[6] = 0

move_group.go(joint_goal, wait=True)

move_group.stop()

# Planning to Pose Goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orienation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)

success = move_group.go(wait=True) # go() return boolean indicatong when plannig and execution was success

move_group.stop()

move_group.clear_pose_target()

# Cartesian Paths
scale = 1.0
waypoints = []
wpose =move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1 # first move up (Z)
wpose.position.y += scale * 0.2 # and sideway (Y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1 # second move forward/backward in (X)
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0 # waypoint to follow
)

# return plan, fraction

# Display a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# Executing Plan
move_group.execute(plan, wait=True)

# Adding Object to Planning Scene
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "AUBO I5"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11 # above the AUBO_hand frame
box_name = "box"

# Ensuring Collision Updates are Recieves
start = rospy.get_time()
seconds = rospy.get_time()
while (seconds - start < timeout) and not rospy.is_shutdown():
    # test if the box is in attached objects
    attached_object = scene.get_attached_objects([box_name])
    is_attached = len(attached_object.keys()) > 0

    # test if box is in the scene
    # note attaching box will remove it from know_object
    is_known = box_name in scene.get_known_object_name()

    # test if we are in expectes state
    if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

    rospy.sleep(0.1)
    seconds = rospy.get_time()

# return False

#Attaching Objects to the Robot
grasping_group = "AUBO_hand"
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)

# Detaching Objects from Robot
scene.remove_attached_object(eef_link, name=box_name)

# Remove Objects from the Planning Scene
scene.remove_world_object(box_name)