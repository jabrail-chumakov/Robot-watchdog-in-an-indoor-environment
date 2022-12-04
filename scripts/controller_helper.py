#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script which helps the controller script to move the robot between rooms.

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
 # Required to pass the `ControlAction` type for instantiating the `SimpleActionServer`.
import robot_watchdog_in_an_indoor_environment.msg 
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER_HELPER

def controller_helper(goal):
    """
    This function allows controller script receive goal coordinates for the robot.
    
    Args:
        goal: Goal points
    """
    # Instantiate and start the action controller client based on the `SimpleActionClient` class.
    helper = SimpleActionClient(anm.ACTION_CONTROLLER,
                                robot_watchdog_in_an_indoor_environment.msg.ControlAction)
    # Waiting for initialization of the server.
    helper.wait_for_server()
    # Communicate with server about robot's goal.
    helper.send_goal(goal)
    timeout = helper.wait_for_result(rospy.Duration(15))
    # If movement to the final goal is finished before timeout
    if timeout:
        log_msg = Fore.GREEN + 'A robot reached the final goal!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
        return helper.get_result()
    # Otherwise
    else:
        log_msg = Fore.RED + 'A robot was not able to reach final goal!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
        helper.cancel_all_goals()
        
def controller_helper_results(results):
    """
    Executes controller_helper(results) function.
    
    Args:
        results: Coordinates for robot
    """
    controller_helper(results)

if __name__ == '__main__':
    # Initialize the node, subscribe, and wait.
    rospy.init_node(anm.NODE_CONTROLLER_HELPER, log_level = rospy.INFO)
    rospy.Subscriber('/path', robot_watchdog_in_an_indoor_environment.msg.PlanResult, controller_helper_results)
    rospy.spin()