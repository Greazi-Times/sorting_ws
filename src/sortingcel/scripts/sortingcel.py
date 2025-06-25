#!/usr/bin/env python3

#Import libraries
import rospy
import ROSHandler

# Messages
from hmi.msg import ControlCommand
from hmi_listener import HMICommandListener
from transport_handler import Transportsystem

import std_msgs
from std_msgs.msg import Bool

state = None
continues = False

class sorting_system:
    """
    Class which handles the whole project
    """

    def __init__(self, rospy, ros_handler, transport_handler, camera_handler=None, robot_handler=None):
        """
        Initialize the sorting system.
        """
        self.rospy = rospy
        self.ros_handler = ros_handler
        self.transport_handler = transport_handler
        self.camera_handler = camera_handler
        self.robot_handler = robot_handler
        self.hmi_handler = HMICommandListener(rospy)
        rospy.loginfo("Sorting system initialized.")
    
    def reset_system(self):
        """
        Reset the sorting system to its initial state.
        """
        global state, continues
        rospy.loginfo("Resetting sorting system...")
        self.hmi_handler.reset_system()
        state = "RESET"
        continues = False
        rospy.loginfo("Sorting system reset complete.")

    def start_system(self):
        self.transport_handler.perform()


class subscribers:
    """
    Class to manage ROS subscribers for the sortingcel system.
    """

    def __init__(self, rospy, hmi_handler, sorting_system):
        """
        Initialize the subscribers.
        """
        self.rospy = rospy
        self.hmi_handler = hmi_handler
        self.sorting_system = sorting_system

    def hmi_command_callback(self, msg):
        """
        Callback function for HMI command messages.
        """
        global state, continues
        rospy.loginfo(f"[HMI Command] Received command: {msg.command}")
        # Process the command here
        # Example: self.ros_handler.process_command(msg)
        
        command_msg = msg.command
        if command_msg == "START":
            self.hmi_handler.start_sorting()
            self.sorting_system.start_system()
        elif command_msg == "STOP":
            self.hmi_handler.stop_system()
        elif command_msg == "RESET":
            self.hmi_handler.reset_system()
        elif command_msg == "EMERGENCY_STOP":
            self.hmi_handler.emergency_stop()
        elif command_msg == "CONSTANT":
            self.hmi_handler.constant = True
        elif command_msg == "SINGLE":
            self.hmi_handler.constant = False
        else:
            rospy.logwarn(f"[HMI Listener] Unknown command: {msg.command}")
            state = "ERROR"

# Main initialization function
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('sortingcel', anonymous=True)
    rospy.loginfo("Starting sortingcel system...")

    # Initialize the handler
    ros_handler = ROSHandler.ROSHandler(rospy)
    rospy.loginfo("ROSHandler initialized.")

    # Create instance of hanlders
    transport_handler = Transportsystem(rospy, ros_handler)

    hmi_handler = HMICommandListener(rospy, ros_handler)
    subscribers = subscribers(rospy, hmi_handler)

    sorting = sorting_system(rospy, ros_handler, hmi_handler, transport_handler)

    # Subscribe using the listener's method
    ros_handler.create_subscriber('hmi/user_command', ControlCommand, subscribers.hmi_command_callback)
    ros_handler.create_subscriber('transportsystem/sensor/start', Bool, transport_handler._beginsensor_callback)
    ros_handler.create_subscriber('transportsystem/sensor/end', Bool, transport_handler._eindsensor_callback)

    # Create publishers
    ros_handler.create_publisher('hmi/system_command', ControlCommand)
    ros_handler.create_publisher('light_indication/constant/on', std_msgs.msg.String)
    ros_handler.create_publisher('light_indication/blink/on', std_msgs.msg.String)
    ros_handler.create_publisher('light_indication/off', std_msgs.msg.String)
    ros_handler.create_publisher('transportsystem/command', Bool)

    rospy.spin()
    rospy.loginfo("Sortingcel system is running. Waiting for commands...")
