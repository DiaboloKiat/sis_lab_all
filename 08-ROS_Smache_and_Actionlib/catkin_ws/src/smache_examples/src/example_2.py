#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random


# SIS Lab 8 smache example 2
# Michael Su Oct 2018

# define state Meet
class Meet(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['My_love','Next_is_better','Give_up'],
            input_keys=['meet_input'],
            output_keys=['meet_output'],
        )
        self.counter = 0
        self.max_meet_time = 10
        self.pursuit_method_list = {}

    def execute(self, userdata):
        self.pursuit_method_list[str(userdata.meet_input)] = userdata.meet_input
        flag_like = random.randint(0,1)
        if self.counter >= self.max_meet_time:
            return 'Give_up'

        if flag_like == 0:
            self.counter = self.counter + 1
            return 'Next_is_better'
        else:
            userdata.meet_output = self.pursuit_method_list
            self.counter = self.counter + 1
            return 'My_love'

# define state Pursuit
class Pursuit(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['Success', 'Fail'],
            input_keys=['pursuit_input'],
            output_keys=['pursuit_output'],
            )


    def execute(self, userdata):
        person_style = random.randint(0,5)
        rospy.loginfo("Person style: %d",person_style)
        # print(person_style)
        # print userdata.pursuit_input
        if str(person_style) in userdata.pursuit_input.keys():
            pursuit_method = userdata.pursuit_input[str(person_style)]
            rospy.loginfo("I have experience.")
        else:
            pursuit_method = random.randint(0,5)
            userdata.pursuit_output = person_style
            rospy.loginfo("No experience.")
        if person_style != pursuit_method:
            return 'Fail'
        else:
            return 'Success'


def main():
    rospy.init_node('Smach_example2')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Perfect_life','Single_is_good'])
    sm.userdata.pursuit_method_list = {}
    sm.userdata.person_style = -1
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            'Meet',
            Meet(),
            transitions={'My_love':'Pursuit', 'Next_is_better':'Meet',
                'Give_up':'Single_is_good'},
            remapping={'meet_input':'person_style','meet_output':'pursuit_method_list'}
        )
        smach.StateMachine.add(
            'Pursuit',
            Pursuit(),
            transitions={'Success':'Perfect_life','Fail':'Meet'},
            remapping={'pursuit_input':'pursuit_method_list','pursuit_output':'person_style'}
        )

    sis = smach_ros.IntrospectionServer('server_name', sm, '/LAB8_EXAMPLE_2')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
