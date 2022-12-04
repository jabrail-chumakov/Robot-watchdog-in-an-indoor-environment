#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script is utilized by the package to define each variable's name.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful
    
"""

import rospy

# The name of the parameter to define the environment size.
# It should be a list `[max_x, max_y]` such that x:[0, `max_x`) and y:[0, `max_y`).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'

# -------------------------------------------------

# The name of the finite state machine node.
NODE_FSM = 'fsm'

# ---------------------------------------------------------

# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot-state'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the server to receive the current robot's charge level.
SERVER_RECEIVE_CHARGE = 'state/receive_charge_level'

# The name of the server to send the current robot's charge level. 
SERVER_SEND_CHARGE = 'state/send_charge_level'

# ---------------------------------------------------------

# The name of the controller node.s
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The name of the controller helper node.
NODE_CONTROLLER_HELPER = 'controller_helper'

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The name of the planner helper node.
NODE_PLANNER_HELPER = 'planner_helper'

# -------------------------------------------------

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    """
    The log tag that is displayed in the finite state machine while it is operating is created using this function.
    
    Args:
        msg(String): Message to display
        producer_tag(String): Name of the state
    Returns:
        log_msg(String): Message to the log
    """
    return '@%s>> %s' % (producer_tag, msg)
