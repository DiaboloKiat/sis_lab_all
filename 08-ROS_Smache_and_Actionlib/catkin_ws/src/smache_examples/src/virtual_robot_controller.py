#!/usr/bin/env python
#Written by YungShan Su in November

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
import actionlib


"""Robot controller

This module provides you functions to manipulate Kaku arm and navigate Kaku to
destimation. For manipulation, it provides joint_space planning, pose planning,
and Cartesian path planning. For navigation, it provides user a ease interface
to assign destimation to navigave.

"""

class Kaku_robot_control(object):

    """Kaku_robot"""

    def __init__(self):
        """Controller initialization

        """
        super(Kaku_robot_control, self).__init__()

        #Initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_planner_id("RRTConnectkConfigDefault")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        joint_goal = group.get_current_joint_values()
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        #Initialize navigation client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


    def go_to_joint_state(
            self,
            joint_goal = [0,0,0,0,0]
        ):
        """Manipulate robot arm to assigned joint

        """
        self.group.set_joint_value_target(joint_goal)
        plan = self.group.plan()
        self.display_trajectory(plan)
        self.group.execute(plan, wait=True)
        return


    def go_to_pose_goal(self, pose_goal):
        """Manipulate robot arm to assigned pose

        """
        self.group.set_pose_target(pose_goal)
        plan = self.group.plan()
        self.display_trajectory(plan)
        self.group.execute(plan, wait=True)
        self.group.clear_pose_targets()
        return


    def plan_cartesian_path(self, x ,y ,z):
        """Manipulate robot arm to follow cartesian_path and move tp
        destimation.

        """
        ## Cartesian Paths
        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.x += x * 0.3  # First move up (z)
        wpose.position.y += y * 0.3  # and sideways (y)
        wpose.position.z += z * 0.3  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += x * 0.6  # Second move forward/backwards in (x)
        wpose.position.y += y * 0.6
        wpose.position.z += z * 0.6
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += x   # Second move forward/backwards in (x)
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0,
                                           avoid_collisions = False ) # jump_threshold
        return plan, fraction


    def display_trajectory(self, plan):
        """Display arm planning trajectory

        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        """Execute planning

        """
        self.group.execute(plan, wait=True)


    def wait_for_state_update(
            self,
            box_is_known=False,
            box_is_attached=False,
            timeout=4):
        """Wait object state to update

        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = self.scene.get_attached_objects([self.box_name])
          is_attached = len(attached_objects.keys()) > 0

          is_known = self.box_name in self.scene.get_known_object_names()
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_box(self, timeout=4):
        """Add box to the map

        """
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "gripper_finger_base_link"
        box_pose.pose.orientation.w = 1.0
        rospy.sleep(0.1)
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.05, 0.05, 0.05))
        # self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        """Attach Object to the Robot

        """
        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, timeout=4):
        """Deattach Object to the Robot

        """
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        """Remove box from the map

        """
        self.scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    def navigate(self, pose):
        """Navigate robot to destimation

        """
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.client.send_goal(goal)
        self.client.wait_for_result()
        print self.client.get_state()
        return


def main():
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    kaku_robot_control = Kaku_robot_control()

    #Manipulate arm to home weight point
    home_joint_pose = [9.45e-05, -1.24, 1.80, 1.42, -2.26e-05]
    kaku_robot_control.go_to_joint_state(home_joint_pose)
    current_pose = kaku_robot_control.group.get_current_pose().pose
    print current_pose

    #Manipulate arm to specific weight point
    plan, fraction = kaku_robot_control.plan_cartesian_path(0.15, 0 , -0.05)
    kaku_robot_control.display_trajectory(plan)
    kaku_robot_control.execute_plan(plan)
    current_pose = kaku_robot_control.group.get_current_pose().pose
    print current_pose

    #Pick object
    kaku_robot_control.add_box()
    kaku_robot_control.attach_box()

    #Manipulate arm back to home weight point
    plan, fraction = kaku_robot_control.plan_cartesian_path(-0.15, 0 , 0.05)
    kaku_robot_control.display_trajectory(plan)
    kaku_robot_control.execute_plan(plan)
    current_pose = kaku_robot_control.group.get_current_pose().pose
    print current_pose

    #Remove object
    kaku_robot_control.detach_box()
    kaku_robot_control.remove_box()

    #Naviagate robot to destimation
    target_destimation = Pose()
    target_destimation.position.x = 0.5
    target_destimation.orientation.w = 1.0
    kaku_robot_control.navigate(target_destimation)

    #Naviagate robot to origin
    target_destimation = Pose()
    target_destimation.position.x = 0
    target_destimation.orientation.w = 1.0
    kaku_robot_control.navigate(target_destimation)

if __name__ == '__main__':
    main()
