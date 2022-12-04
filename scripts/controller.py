#! /usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script allows the robot to move between rooms and also controls the battery level.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful
    
"""

import rospy
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
import robot_watchdog_in_an_indoor_environment  
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm
# Import custom message, actions and services.
from robot_watchdog_in_an_indoor_environment.msg import ControlFeedback, ControlResult, Point
from robot_watchdog_in_an_indoor_environment.srv import SetPose, GetPose, Battery 

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER
    
# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):
    def __init__(self):
        """
        This funstion is used to start the action server.
        
        Args:
            none
            
        Returns:
            none
        """
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      robot_watchdog_in_an_indoor_environment.msg.ControlAction,
                                      execute_cb = self.execute_callback,
                                      auto_start = False)
        self._as.start()
        # Log information.
        log_msg = (Fore.GREEN + f'Action Server `{anm.ACTION_CONTROLLER}` initialised. It will navigate trough the plan with a delay ' 
                   f'between each via point spanning.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):
        """
        This callback function is used for moving a robot with a given goal point.
        It reads the plan from the read point and set the new position for the robot each time.
        
        Args:
            none
            
        Returns:
            none
        """
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            robot_position = Point()
            robot_position = get_pose()
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            set_pose(point)
            discharge_step()

        # Log current robot position.
        log_msg = f'A robot is moving to ({point.x}, {point.y}) coordinates.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log(Fore.GREEN + 'Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        print('\033[39m') # Makes text from Green to normal
        return  # Succeeded.

# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def set_pose(pose):
    """
    Function that updates the current robot's position stored in the `robot-state` node.
    
    Args:
        pose: Pose
        
    Returns:
        none
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def get_pose():
    """
    Obtain current robot position stored in the `robot-state` node.
    
    Returns:
        pose: Pose
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def charge_sender(charge_level):
    """
    Function that updates the current robot's charge level stored in the `robot-state` node.
    
    Args:
        charge_level(Int): The charge level
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SEND_CHARGE)
    try:
        # Call the service and send the current robot charge capacity.
        service = rospy.ServiceProxy(anm.SERVER_SEND_CHARGE, Battery)
        service(charge_level)
    except rospy.ServiceException as e:
        log_msg = f'Server cannot send actual charge state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
        
def charge_receiver():
    """
    Obtain current robot charge level stored in the `robot-state` node.
    
    Returns:
        battery_level(Int): The charge level
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_RECEIVE_CHARGE)
    try:
        # Call the service and receieve the current robot charge capacity.
        service = rospy.ServiceProxy(anm.SERVER_RECEIVE_CHARGE, Battery)
        charge_level = service().charge_level
        return charge_level
    except rospy.ServiceException as e:
        log_msg = f'Server cannot receive actual charge state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def discharge_step():
    """
    A function that consumes the charge level for each step robot moves between the rooms.
    """
    charge_level = charge_receiver()
    charge_level = charge_level - 5
    charge_sender(charge_level)

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level = rospy.INFO)
    server = ControllingAction()
    rospy.spin()
