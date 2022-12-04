#!/usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python script representing controller
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This script updates the ontology obtains the target room and sets the states and transitions for a finite state machine.

Packages:
    `Colorama <https://pypi.org/project/colorama/>`_: Allows you to make output text colorful

Publishes to:
    /point
"""

import time
import math
import rospy
import smach
import smach_ros
from progress.bar import ShadyBar
from os.path import dirname, realpath
# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style
# Required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
import robot_watchdog_in_an_indoor_environment
# Import custom message, map, actions and services.
from robot_watchdog_in_an_indoor_environment.msg import PlanGoal
from robot_watchdog_in_an_indoor_environment.srv import Battery
from robot_watchdog_in_an_indoor_environment.topological_map import TopologicalMap
# Import constant name defined to structure the architecture.
from robot_watchdog_in_an_indoor_environment import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FSM

def goal_execute(x, y):
    """
    Publishes goal positions for x and y.
    
    Args:
        x(Float): X position
        y(Float): Y position
    """
    goal = PlanGoal()
    goal.target.x, goal.target.y = x, y
    plan_publisher.publish(goal)
    
def room_switcher(room_number):
    """
    A function that manages which goal execution to run depending on the room.
    
    Returns:
        goal_execute: Goal to which move the robot
    """
    return {
                 'E': lambda: goal_execute(7.0, 2.0),
                'C1': lambda: goal_execute(6.0, 9.0),
                'R1': lambda: goal_execute(2.0, 9.0),
                'R2': lambda: goal_execute(2.0, 16.0),
                'C2': lambda: goal_execute(13, 9.0),
                'R3': lambda: goal_execute(18.0, 9.0),
                'R4': lambda: goal_execute(18.0, 16.0)
    }.get(room_number,lambda: None)()
    
def location_managing(room):
    """
    A function that manages to which room the robot should move.
    """
    log_msg = 'The robot is heading into the ' + room
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    room_switcher(room)

def charge_sender(charge_level):
    """
    This function creates a service and sends the current charge level.
    
    Args:
        charge_level(Int): Charge level
    """
    rospy.wait_for_service(anm.SERVER_SEND_CHARGE)
    try:
        service = rospy.ServiceProxy(anm.SERVER_SEND_CHARGE, Battery)
        service(charge_level)
    except rospy.ServiceException as e:
        log_msg = Fore.RED + f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
        print('\033[39m') # Makes text from Green to normal
    
# Conditions for Room E
class RoomE(smach.State):
    """
    Conditions for Room E.
    """
    def __init__(self):
        self.counter = 0
        smach.State.__init__(self, outcomes = ['D7', 'D6', 'Remain'])
        
    def execute(self, userdata):
        """ 
        This function put a robot on battery charge and sends the target
        room to the planner helper together with updating ontology.
        
        Returns:
            'D7'(String): Door 7
            'D6'(String): Door 6
            'Remain'(String): Remain in the same room
        """
        if self.counter != 0:
            with ShadyBar('Charging', max = 100) as bar:
                for i in range(100):
                    time.sleep(0.05)
                    bar.next()
        self.counter += 1
        charge_sender(100)
        log_msg = Fore.GREEN + f'Battery fully charged!'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        time.sleep(2)
        print('\033[39m') # Makes text from Green to normal
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room in ('C1', 'R1', 'R2'):
            location_managing('C1')
            return 'D7'
        elif target_room in ('C2', 'R3', 'R4'):
            location_managing('C2')
            return 'D6'
        else:
            return 'Remain'

# Conditions for Corridor one
class Corridor1(smach.State):
    """
    Conditions for Corridor 1.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D1', 'D2', 'D5', 'D7', 'Remain'])

    def execute(self, userdata):
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D1'(String): Door 1
            'D2'(String): Door 2
            'D5'(String): Door 5
            'D7'(String): Door 7
            'Remain'(String): Remain in the same room
        """
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room in ('R1'):
            location_managing('R1')
            return 'D1'
        elif target_room in ('R2'):
            location_managing('R2')
            return 'D2'
        elif target_room in ('C2', 'R3', 'R4'):
            location_managing('C2')
            return 'D5'
        elif target_room in ('E'):
            location_managing('E')
            return 'D7'
        else:
            return 'Remain'
            
# Conditions for Room 1
class Room1(smach.State):
    """
    Conditions for Room 1.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D1', 'Remain'])
        
    def execute(self, userdata):
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D1'(String): Door 1
            'Remain'(String): Remain in the same room
        """
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room not in ('R1'):
            location_managing('C1')
            return 'D1'
        else:
            return 'Remain'

# Conditions for Room 2
class Room2(smach.State):
    """
    Conditions for Room 2.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D2', 'Remain'])
        
    def execute(self, userdata):
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D2'(String): Door 2
            'Remain'(String): Remain in the same room
        """
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room not in ('R2'):
            location_managing('C1')
            return 'D2'
        else:
            return 'Remain'

# Conditions for Corridor two
class Corridor2(smach.State):
    """
    Conditions for Corridor 2.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D3', 'D4', 'D5', 'D6', 'Remain'])

    def execute(self, userdata):  
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D3'(String): Door 3
            'D4'(String): Door 4
            'D5'(String): Door 5
            'D6'(String): Door 6
            'Remain'(String): Remain in the same room
        """   
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room in ('R3'):
            location_managing('R3')
            return 'D3'
        elif target_room in ('R4'):
            location_managing('R4')
            return 'D4'
        elif target_room in ('C1', 'R1', 'R2'):
            location_managing('C1')
            return 'D5'
        elif target_room in ('E'):
            location_managing('E')
            return 'D6'
        else:
            return 'Remain'
            
# Conditions for Room 3
class Room3(smach.State):
    """
    Conditions for Room 3.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D3', 'Remain'])
        
    def execute(self, userdata):
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D3'(String): Door 3
            'Remain'(String): Remain in the same room
        """
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room not in ('R3'):
            location_managing('C2')
            return 'D3'
        else:
            return 'Remain'

# Conditions for Room 4
class Room4(smach.State):
    """
    Conditions for Room 4.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['D4', 'Remain'])
        
    def execute(self, userdata):
        """ 
        This function sends the target room to the planner helper together with updating ontology.
        
        Returns:
            'D4'(String): Door 4
            'Remain'(String): Remain in the same room
        """
        time.sleep(2)
        current_time = rospy.get_rostime()
        target_room = topological_map.update_ontology(current_time)
        if target_room not in ('R4'):
            location_managing('C2')
            return 'D4'
        else:
            return 'Remain'

if __name__ == '__main__':
    global topological_map
    global plan_publisher
    
    # Initialize node 
    rospy.init_node(LOG_TAG, log_level = rospy.INFO)
    current_time = rospy.get_rostime()
    topological_map = TopologicalMap(LOG_TAG, current_time)

    # Publishes target goal to the planner
    plan_publisher = rospy.Publisher('/point', robot_watchdog_in_an_indoor_environment.msg.PlanGoal, queue_size = 10)

    # Creates a Smach State Machine
    smach_state_machine = smach.StateMachine(outcomes = [])
    smach_state_machine.userdata.sm_counter = 0

    # Opens the Smach State Machine with all conditions considered
    with smach_state_machine:
        smach.StateMachine.add('E', RoomE(), transitions = {'D7':'C1', 'D6':'C2', 'Remain':'E'})
        smach.StateMachine.add('C1', Corridor1(), transitions = {'D7':'E', 'D1':'R1', 'D2':'R2', 'D5':'C2', 'Remain':'C1'})
        smach.StateMachine.add('C2', Corridor2(), transitions = {'D6':'E', 'D3':'R3', 'D4':'R4', 'D5':'C1', 'Remain':'C2'})
        smach.StateMachine.add('R1', Room1(), transitions = {'D1':'C1', 'Remain':'R1'})
        smach.StateMachine.add('R2', Room2(), transitions = {'D2':'C1', 'Remain':'R2'})
        smach.StateMachine.add('R3', Room3(), transitions = {'D3':'C2', 'Remain':'R3'})
        smach.StateMachine.add('R4', Room4(), transitions = {'D4':'C2', 'Remain':'R4'})

    # Run viewer, and starts smach state machine
    sis = smach_ros.IntrospectionServer('server_name', smach_state_machine, '/SM_ROOT')
    sis.start()
    outcome = smach_state_machine.execute()
    rospy.spin()
    sis.stop()