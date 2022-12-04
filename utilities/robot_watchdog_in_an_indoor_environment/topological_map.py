#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script is used in order to work with topological map with ``armor_client.py``.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful
    
"""

import rospy
from os.path import dirname, realpath
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Import the ArmorClient.
from armor_api.armor_client import ArmorClient
# Import custom message, actions and services.
from robot_watchdog_in_an_indoor_environment.msg import Point
from robot_watchdog_in_an_indoor_environment.srv import GetPose, Battery
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm

class TopologicalMap:
    """
    Topological map with ontology implementation class.
    """
    def __init__(self, log_tag, init_time):
        self.log_tag = log_tag
        self.init_time = init_time
        # Set path to topological map and creates it
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../../map/"
        self.client = ArmorClient("ontology", "ontology_reference")
        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "", True, "PELLET", False)
        self.client.utils.set_log_to_terminal(True)
        self.add_individuals_to_classes()
        self.disjoint_individuals()
        self.assign_doors_to_rooms()
        self.add_last_visit_times()
        self.add_robot()

    def add_individuals_to_classes(self):
        """
        Adding rooms, corridors, doors, and a robot to the topological map.
        """
        # Adding of each room into the class
        self.client.manipulation.add_ind_to_class("E", "ROOM")
        self.client.manipulation.add_ind_to_class("C1", "ROOM")
        self.client.manipulation.add_ind_to_class("C2", "ROOM")
        self.client.manipulation.add_ind_to_class("R1", "ROOM")
        self.client.manipulation.add_ind_to_class("R2", "ROOM")
        self.client.manipulation.add_ind_to_class("R3", "ROOM")
        self.client.manipulation.add_ind_to_class("R4", "ROOM")
        self.client.utils.sync_buffered_reasoner()  

        # Adding of each door between rooms into the class
        self.client.manipulation.add_ind_to_class("D1", "DOOR")
        self.client.manipulation.add_ind_to_class("D2", "DOOR")
        self.client.manipulation.add_ind_to_class("D3", "DOOR")
        self.client.manipulation.add_ind_to_class("D4", "DOOR")
        self.client.manipulation.add_ind_to_class("D5", "DOOR")
        self.client.manipulation.add_ind_to_class("D6", "DOOR")
        self.client.manipulation.add_ind_to_class("D7", "DOOR")
        self.client.utils.sync_buffered_reasoner()  

        # Adding of robot into the class
        self.client.manipulation.add_ind_to_class("Robot", "ROBOT")   
        self.client.utils.sync_buffered_reasoner()   

    def disjoint_individuals(self):
        """
        Setting up of transitions between rooms and doors.
        """
        self.client.manipulation.disj_inds_of_class("ROOM")
        self.client.manipulation.disj_inds_of_class("DOOR")
        self.client.utils.sync_buffered_reasoner()

    def assign_doors_to_rooms(self):
        """
        Function that assigns each door to each room.
        """
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
        self.client.utils.sync_buffered_reasoner()

    def add_last_visit_times(self):
        """
        Function for managing last visit time.
        """
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Int", str(self.init_time.secs - 35))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Int", str(self.init_time.secs - 40))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Int", str(self.init_time.secs - 25))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Int", str(self.init_time.secs - 15))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "E", "Int", str(self.init_time.secs))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C1", "Int", str(self.init_time.secs - 5))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C2", "Int", str(self.init_time.secs - 10))  
        self.client.utils.sync_buffered_reasoner()

    def add_robot(self):
        """
        A function that adds a robot in a particular room in current with charge level and threshold for turning back.
        """
        self.client.manipulation.add_dataprop_to_ind("current_time", "Robot", "Int", str(self.init_time.secs))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_objectprop_to_ind("isIn", "Robot", self.get_location())
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("batteryLvl", "Robot", "Int", str(self.receive_charge_level_client()))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot", "Int", "20")
        self.client.utils.sync_buffered_reasoner()       

    def cut_dataprop(self, data_prop):
        """ 
        A function that removes the data properties.
        
        Returns:
            data_prop(String): Data properties
        """
        start = 0
        end = data_prop.rfind('^') - 2
        data_prop = data_prop[(start + 1) : end]
        return data_prop

    def cut_dataprop_list(self, data_prop_list):
        """
        A function that removes the data properties from the list.
        
        Returns:
            data_prop_list: Data properties list
        
        """
        for i in range(len(data_prop_list)):
            data_prop_list[i] = self.cut_dataprop(data_prop_list[i])
        return data_prop_list

    def receive_charge_level_client(self):
        """
        A function receives charge level.
        
        Returns:
            charge_level(Int): Charge level
        """
        rospy.wait_for_service(anm.SERVER_RECEIVE_CHARGE)
        try:
            service = rospy.ServiceProxy(anm.SERVER_RECEIVE_CHARGE, Battery)
            response = service()
            charge_level = response.charge_level
            return charge_level
        except rospy.ServiceException as e:
            log_msg = Fore.RED + f'Server cannot get current robot battery level: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))
            print('\033[39m') # Makes text from Red to normal

    def get_pose_client(self):
        """
        A function that receives pose.
        
        Returns:
            pose: Pose
        """
        rospy.wait_for_service(anm.SERVER_GET_POSE)
        try:
            service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
            response = service()
            pose = response.pose
            return pose
        except rospy.ServiceException as e:
            log_msg = Fore.RED + f'Server cannot get current robot position: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))
            print('\033[39m') # Makes text from Red to normal

    def get_location(self):
        """
        A function that checks where the robot is now and updates ontology in correspondence with its location.
        
        Returns:
            is_in(String): Is in which room
        """
        pose = Point()
        pose = self.get_pose_client()
        current_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("current_time", "Robot"))[0]

        if (pose.y <= 4):
            is_in = "E"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "E"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "E", "Int", current_time, prev_time)
            
        elif (pose.x <= 5) and (4 < pose.y <= 12):
            is_in = "R1"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R1", "Int", current_time, prev_time)
            
        elif (pose.x <= 5) and (pose.y > 12):
            is_in = "R2"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R2", "Int", current_time, prev_time)

        elif  (5 < pose.x <= 10) and (pose.y > 4):
            is_in = "C1"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "C1", "Int", current_time, prev_time)

        elif (10 < pose.x <= 14.5) and (pose.y > 4):
            is_in = "C2"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C2"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "C2", "Int", current_time, prev_time)

        elif (pose.x > 14.5) and (4 < pose.y <= 12):
            is_in = "R3"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R3", "Int", current_time, prev_time)

        elif (pose.x > 14.5) and (pose.y > 12):
            is_in = "R4"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R4", "Int", current_time, prev_time)

        self.client.utils.sync_buffered_reasoner()
        return is_in


    def update_ontology(self, current_time):
        """
        A function that updates ontology with managing threshold for the last visited room and charge level.
        
        Returns:
            target_room(String): Target room
        """
        # Update robot time instance
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("current_time", "Robot"))[0]
        self.client.manipulation.replace_dataprop_b2_ind("current_time", "Robot", "Int", str(current_time.secs), prev_time)
        self.client.utils.sync_buffered_reasoner()
        # Update battery level
        prevcharge_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        charge_level = str(self.receive_charge_level_client())
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", charge_level, prevcharge_level)
        self.client.utils.sync_buffered_reasoner()
        log_msg = 'Battery level: ' + self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0] + '%'
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        # Update robot location
        loc = self.get_location()
        log_msg = 'Previous room: ' + loc
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))

        visitedAt_E = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "E"))[0]
        visitedAt_R1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
        visitedAt_R2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
        visitedAt_R3 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
        visitedAt_R4 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
        visitedAt_C1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
        visitedAt_C2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C2"))[0]

        visitedAt_dict = {visitedAt_R1: "R1", visitedAt_R2: "R2", visitedAt_R3: "R3", visitedAt_R4: "R4", visitedAt_C1: "C1", visitedAt_C2: "C2", visitedAt_E: "E"}
        visitedAt_dict = dict(sorted(visitedAt_dict.items()))
        room_list = list(visitedAt_dict.values())
        target_room = room_list[0]

        urgency_threshold = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("urgencyThreshold", "Robot"))[0]

        if current_time.secs - int(visitedAt_E) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("E", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("E", "URGENT")

        if current_time.secs - int(visitedAt_R1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R1", "URGENT")
        
        if current_time.secs - int(visitedAt_R2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R2", "URGENT")
        
        if current_time.secs - int(visitedAt_R3) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R3", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R3", "URGENT")
        
        if current_time.secs - int(visitedAt_R4) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R4", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R4", "URGENT")
        
        if current_time.secs - int(visitedAt_C1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C1", "URGENT")
        
        if current_time.secs - int(visitedAt_C2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C2", "URGENT")
        self.client.utils.sync_buffered_reasoner()

        urgent_rooms = self.client.query.ind_b2_class("URGENT")
        
        battery_lvl = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        if int(battery_lvl) > int(urgency_threshold):
            log_msg = Fore.CYAN + 'Target room: ' + target_room
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            print('\033[39m') # Makes text from Cyan to normal
            return target_room
        else:
            log_msg = Fore.CYAN + 'Target room: "E"' 
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            print('\033[39m') # Makes text from Cyan to normal
            return "E"

