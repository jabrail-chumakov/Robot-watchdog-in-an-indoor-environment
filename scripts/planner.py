#! /usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script is used for planning the motion.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful
    
"""

import rospy
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm
# Import custom message, actions and services.
from robot_watchdog_in_an_indoor_environment.srv import GetPose
from robot_watchdog_in_an_indoor_environment.msg import Point, PlanFeedback, PlanResult
# Required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
import robot_watchdog_in_an_indoor_environment  

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER
        
# An action server to simulate motion planning.
# Given a target position, it retrieve the current robot position from the 
# `robot-state` node, and return a plan as a set of via points.
class PlaningAction(object):
    """
    A class that is used to plan goals from start to target.
    """
    def __init__(self):
        # Get random-based parameters used by this server
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      robot_watchdog_in_an_indoor_environment.msg.PlanAction, 
                                      execute_cb = self.execute_callback, 
                                      auto_start = False)
        self._as.start()
        
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
                   f'spanning in planner points. Each point will be generated '
                   f'with a delay spanning in planner time.')
        
    def execute_callback(self, goal):
        """
        This function checks if the start and target positions are correct and then publishes the results to the client.
        
        Args:
            goal: Goal
        """
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = get_pose()
        target_point = goal.target

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_point is None or target_point is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            log_msg = (f'Start point ({start_point.x}, {start_point.y}) or target point ({target_point.x}, '
                       f'{target_point.y}) point out of the environment. This service will be aborted!.')
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        feedback.via_points.append(target_point)
        
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        log_msg = 'Server is planning points...'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = Fore.GREEN + 'Motion plan succeeded with plan (starting, target): ' + Fore.YELLOW 
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + ')' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
        
    # Check if the point is within the environment bounds, i.e.
    # x: [0, `self._environment_size[0]`], and y: [0, `self._environment_size[1]`].
    def _is_valid(self, point):
        """
        Check if the point is within the environment bounds.
        """
        return 0.0 <= point.x <= self._environment_size[0] and 0.0 <= point.y <= self._environment_size[1]

# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def get_pose():
    """
    This function receives the current position from the server get the pose.
    
    Returns:
        pose: Point
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        return pose
    except rospy.ServiceException as e:
        log_msg = Fore.RED + f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level = rospy.INFO)
    server = PlaningAction()
    rospy.spin()
