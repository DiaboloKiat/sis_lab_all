#! /usr/bin/env python
# Revised by YungshanSu, in November
"""Action client example

In this code, it demonstrates how to initialize action client, and defines
feedback and goal callback function.
"""

import rospy
import actionlib
import action_example.msg
from actionlib_msgs.msg import GoalStatus

def do_dishes_client(dish_number = 20, execution_time = 5):
    """Send goal to server and wait server to execute tasks in specific period

    Args:
        dish_number: number of dishes
        execution_time: the time server needs to finish tasks

    Returns:
        DoDishesResult: the total number of dishes which server finied doing
    """
    # Creates the SimpleActionClient, passing the type of the action
    # (DoDishesAction) to the constructor.
    client = actionlib.SimpleActionClient(
        '/do_dishes_server',
        action_example.msg.DoDishesAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = action_example.msg.DoDishesGoal(dishwasher_goal=dish_number)
    exe_time = rospy.Duration(execution_time)

    # Send goal to server and
    client.send_goal(
        goal,
        done_cb=do_dishes_done,
        feedback_cb=do_dishes_feedback)

    #Wait server to execute tasks for exe_time.
    client.wait_for_result (timeout=exe_time)

    #Get recent state
    action_state = client.get_state()
    if action_state != GoalStatus.SUCCEEDED:
        client.cancel_goal()
    return


def do_dishes_feedback(feedback):
    """Feedback callback function

    """
    rospy.loginfo("Robot finished %d dishes." % feedback.total_dishes_cleaned)


def do_dishes_done(state, result):
    """Goal callback function

    """
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Robot finished task.")
    else:
        rospy.loginfo("Robot fails.")


def main():
    rospy.init_node('do_dishes_client')
    while True:
        try:
            dish_number = int(raw_input("How many dishes do you have?: "))
            exec_time = float(raw_input("How long do you expect robot to finish this task?: "))
            do_dishes_client(dish_number, exec_time)
            rospy.sleep(0.5)
        except Exception:
            print (Exception)
            continue

if __name__ == '__main__':
    main()
