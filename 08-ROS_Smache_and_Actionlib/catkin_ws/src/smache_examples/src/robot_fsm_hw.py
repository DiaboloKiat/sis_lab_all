#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random
from virtual_robot_controller import Kaku_robot_control
from geometry_msgs.msg import Pose
# SIS Lab 8 robot fsm example
#Written by YungShan Su in November


# define state Meet
class Pick(smach.State):
    def __init__(self, robot_controller):
        smach.State.__init__(self, outcomes=['Success','Fail'])
        self.controller = robot_controller

    def execute(self, userdata):
        try:
            #Manipulate arm to home weight point
            home_joint_pose = [9.45e-05, -1.24, 1.80, 1.42, -2.26e-05]
            self.controller.go_to_joint_state(home_joint_pose)
            #Manipulate arm to specific weight point
            plan, fraction = self.controller.plan_cartesian_path(0.15, 0 , -0.05)
            # self.controller.display_trajectory(plan)
            self.controller.execute_plan(plan)
            #Pick object
            self.controller.add_box()
            self.controller.attach_box()
            #Manipulate arm to home weight point
            plan, fraction = self.controller.plan_cartesian_path(-0.15, 0 , 0.05)
            # self.controller.display_trajectory(plan)
            self.controller.execute_plan(plan)
            return 'Success'
        except:
            return 'Fail'

# define state Pursuit
class Move_to(smach.State):
    def __init__(self, robot_controller):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.controller = robot_controller

    def execute(self, userdata):
        try:
            target_destimation = Pose()
            target_destimation.position.x = 0.5
            target_destimation.orientation.w = 1.0
            self.controller.navigate(target_destimation)
            return 'Success'
        except:
            return 'Fail'


# define state Pursuit
class Place(smach.State):
    def __init__(self, robot_controller):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.controller = robot_controller

    def execute(self, userdata):
        try:
            #Manipulate arm to home weight point
            home_joint_pose = [9.45e-05, -1.24, 1.80, 1.42, -2.26e-05]
            self.controller.go_to_joint_state(home_joint_pose)
            #Manipulate arm to specific weight point
            plan, fraction = self.controller.plan_cartesian_path(0.15, 0 , -0.05)
            # self.controller.display_trajectory(plan)
            self.controller.execute_plan(plan)
            #Place object
            self.controller.detach_box()
            self.controller.remove_box()
            #Manipulate arm to home weight point
            plan, fraction = self.controller.plan_cartesian_path(-0.15, 0 , 0.05)
            # self.controller.display_trajectory(plan)
            self.controller.execute_plan(plan)
            return 'Success'
        except:
            return 'Fail'


# define state Pursuit
class Move_back(smach.State):
    def __init__(self, robot_controller):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])
        self.controller = robot_controller
    def execute(self, userdata):
        try:
            target_destimation = Pose()
            target_destimation.position.x = 0
            target_destimation.orientation.w = 1.0
            self.controller.navigate(target_destimation)
            return 'Success'
        except:
            return 'Fail'


def main():
    rospy.init_node('Robot_smach_example')
    kaku_robot_control = Kaku_robot_control()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])
    sm.userdata.finish_action = False
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Pick', Pick(kaku_robot_control),
            transitions={'Fail':'Pick', 'Success':'Move_to'})
        smach.StateMachine.add('Move_to', Move_to(kaku_robot_control),
            transitions={'Fail':'Move_to', 'Success':'Place'})
        smach.StateMachine.add('Place', Place(kaku_robot_control),
            transitions={'Fail':'Place', 'Success':'Move_back'})
        smach.StateMachine.add('Move_back', Move_back(kaku_robot_control),
            transitions={'Fail':'Move_back', 'Success':'Pick'})
    sis = smach_ros.IntrospectionServer('server_name', sm, '/robot_fsm_example')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
