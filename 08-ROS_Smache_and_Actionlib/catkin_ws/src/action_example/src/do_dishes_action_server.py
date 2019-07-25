#! /usr/bin/env python
# Revised by YungshanSu, in November
"""Action server example

In this code, it demonstrates how to initialize action server, and defines
execute callback function.
"""


import rospy
import actionlib
import action_example.msg

class DoDishesAction (object):
    """Action server for doing dishes

    """
    def __init__(self, name):
        self._feedback = action_example.msg.DoDishesFeedback()
        self._result = action_example.msg.DoDishesResult()
        self.goal_dishes = 0

        self._action_name = name
        #Initialize action server
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            action_example.msg.DoDishesAction,
            execute_cb=self.execute_cb,
            auto_start = False)
        rospy.loginfo("Server %s is initialized."  % self._action_name)
        #Start action server
        self._as.start()
        rospy.loginfo("Server %s can be used now." % self._action_name)

    def execute_cb(self, goal):
        # Start executing the action
        self.goal_dishes = goal.dishwasher_goal
        rospy.loginfo("Start to do %d dishes" % self.goal_dishes)
        count = 0
        for i in range (0, self.goal_dishes):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._result.total_dishes_cleaned = count
                self._as.set_preempted()
                break
            else:
                count += 1
                self._feedback.total_dishes_cleaned = count
                self._as.publish_feedback(self._feedback)
                rospy.sleep(0.5)

        if count == self.goal_dishes:
            self._result.total_dishes_cleaned = count
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

def main():

    rospy.init_node('do_dishes_server')
    server = DoDishesAction(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()
