#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script is used for managing robot states.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful
    
"""

import rospy
import random
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from robot_watchdog_in_an_indoor_environment.msg import Point
from robot_watchdog_in_an_indoor_environment.srv import GetPose, SetPose, Battery, GetPoseResponse, SetPoseResponse, BatteryResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE

# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:
    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level = rospy.INFO)
        # Initialise robot position
        self._pose = Point()
        point_coord = rospy.get_param('/state/initial_pose')
        self._pose.x, self._pose.y = point_coord[0], point_coord[1]
        # Initialise charge level
        self.charge_level = 100
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        rospy.Service(anm.SERVER_RECEIVE_CHARGE, Battery, self.receive_charge_level)
        rospy.Service(anm.SERVER_SEND_CHARGE, Battery, self.send_charge_level)
        # Log information.
        # log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
        #            f'`{anm.SERVER_SET_POSE}`, `{anm.SERVER_RECEIVE_CHARGE}` and `{anm.SERVER_SEND_CHARGE}`.')
        # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_pose(self, request):
        """
        A function that sends a response about the robot's position.
        
        Returns:
            SetPoseResponse(): Set Pose Response
        """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            rospy.loginfo(anm.tag_log(Fore.GREEN + f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                                      f'as ({self._pose.x}, {self._pose.y}).', LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        else:
            rospy.logerr(anm.tag_log(Fore.RED + 'Cannot set an unspecified robot position', LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        # Return an empty response.
        return SetPoseResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_pose(self, request):
        """
        A function that receives a response from the robot about position.
        
        Returns:
            response: Response
        """
        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log(Fore.RED + 'Cannot get an unspecified robot position', LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        else:
            log_msg = Fore.YELLOW + f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def send_charge_level(self, request):
        """
        A function that sends a response about the robot's charge level.
        
        Returns:
            BatteryResponse(): Battery response
        """
        if request.charge_level is not None:
            # Store the new current robot charge level.
            self.charge_level = request.charge_level
            # Log information.
            log_msg = (Fore.MAGENTA + f'Set current robot charge level through `{anm.SERVER_SEND_CHARGE}` '
                       f'as ({self.charge_level}).')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        else:
            rospy.logerr(anm.tag_log(Fore.RED + 'Cannot set an unspecified robot charge level', LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        # Return an empty response.
        return BatteryResponse()

    def receive_charge_level(self, request):
        """
        A function that receives a response from the robot about charge level.
        
        Returns:
            respose: Response
        """
        # Log information.
        if self.charge_level is None:
            rospy.logerr(anm.tag_log(Fore.RED + 'Cannot get an unspecified robot charge level', LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        else:
            log_msg = Fore.CYAN + f'Get current robot charge level through `{anm.SERVER_RECEIVE_CHARGE}` as ({self.charge_level})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print('\033[39m') # Makes text from Green to normal
        # Create the response with the robot battery level and return it.
        response = BatteryResponse()
        response.charge_level = self.charge_level
        return response
    
if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

