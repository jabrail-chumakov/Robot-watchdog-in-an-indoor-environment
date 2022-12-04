#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script is used for helping in motion planning.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful

Subscribes to:
    /path
"""

import rospy
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Import the ActionServer implementation used.
from actionlib import SimpleActionClient
# Required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
import robot_watchdog_in_an_indoor_environment 
# Import custom message
from robot_watchdog_in_an_indoor_environment.msg import PlanGoal
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER_HELPER

def planner_helper(x, y):
    """
    This function sends the target goal to the planner script.
    
    Args:
        x(Float): X position
        y(Float): Y position
    """
    # Instantiate and start the action planner client based on the `SimpleActionClient` class.
    helper = SimpleActionClient(anm.ACTION_PLANNER,
                                robot_watchdog_in_an_indoor_environment.msg.PlanAction)
    # Waiting for initialization of the server.
    helper.wait_for_server()
    goal = PlanGoal()
    goal.target.x, goal.target.y  = x, y
    # Communicate with server about robot's goal.
    helper.send_goal(goal)
    timeout = helper.wait_for_result(rospy.Duration(15))
    # If movement plan to the final goal is obtained before timeout
    if timeout:
        log_msg = Fore.GREEN + 'A movement plan is obtained!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
        return helper.get_result()
        # Otherwise
    else:
        log_msg = Fore.RED + 'A helper was not able to receieve movement plan!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
        helper.cancel_all_goals()
        
def planner_helper_results(results):
    """
    This function publishes planner helper so the planner script can receive info about target x and y coordinates.
    
    Args:
        results: Coordinates for robot
    """
    global plan_publisher
    plan_publisher.publish(planner_helper(results.target.x, results.target.y))

if __name__ == '__main__':
    global plan_publisher
    rospy.init_node(anm.NODE_PLANNER_HELPER, log_level = rospy.INFO)
    rospy.Subscriber('/point', robot_watchdog_in_an_indoor_environment.msg.PlanGoal, planner_helper_results)
    plan_publisher = rospy.Publisher('/path', robot_watchdog_in_an_indoor_environment.msg.PlanResult, queue_size = 10)
    rospy.spin()