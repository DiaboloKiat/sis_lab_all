#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random


# SIS Lab 8 smache example 1
# Michael Su Oct 2018


# define state Meet
class Meet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['My_love','Next_is_better','Give_up'])
        self.counter = 0
        self.max_meet_time = 5
    def execute(self, userdata):
        flag_like = random.randint(0,1)
        if self.counter >= self.max_meet_time:
            return 'Give_up'

        if flag_like == 0:
            self.counter = self.counter + 1
            return 'Next_is_better'
        else:
            self.counter = self.counter + 1
            return 'My_love'

# define state Pursuit
class Pursuit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Fail'])


    def execute(self, userdata):
        person_style = random.randint(0,2)
        rospy.loginfo("Person style: %d",person_style)
        pursuit_method = random.randint(0,2)
        if person_style != pursuit_method:
            return 'Fail'
        else:
            return 'Success'


def main():
    rospy.init_node('Smach_example1')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Perfect_life','Single_is_good'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Meet', Meet(),
            transitions={'My_love':'Pursuit', 'Next_is_better':'Meet',
            'Give_up':'Single_is_good'})
        smach.StateMachine.add('Pursuit', Pursuit(),
            transitions={'Success':'Perfect_life','Fail':'Meet'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/LAB8_EXAMPLE_1')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
