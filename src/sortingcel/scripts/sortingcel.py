#!/usr/bin/env python3

#Import libraries
import os
import rospy
import ROSHandler

# Messages
from hmi.msg import ControlCommand
import hmi_listener

import std_msgs

class subscribers:
    """
    Class to manage ROS subscribers for the sortingcel system.
    """

    def HmiCommandCallback(self, msg):
        """
        Callback function for HMI commands.
        This function will be called whenever a new command is received from the HMI.
        """
        # Implement command handling logic here
        # Example: self.handler.handle_hmi_command(msg)
        command = msg.command
        rospy.loginfo(f"[HMI Command] Received command: {msg.command}")

        command = command.upper()
        # Handle different commands
        if command == "START":
            rospy.loginfo("[Action] Starting sorting process...")
        elif command == "STOP":
            rospy.loginfo("[Action] Stopping sorting process...")
        elif command == "RESET":
            rospy.loginfo("[Action] Resetting system...")
        elif command == "EMERGENCY_STOP":
            rospy.loginfo("[Action] Emergency stop triggered!")
        elif command == "CONSTANT":
            rospy.loginfo("[Action] Setting to constant mode.")
        elif command == "SINGLE":
            rospy.loginfo("[Action] Setting to single mode.")
        else:
            rospy.logwarn(f"[HMI Command] Unknown command: {msg.command}")


# Main initialization function
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('sortingcel', anonymous=True)
    rospy.loginfo("Starting sortingcel system...")

    # Initialize the handler
    handler = ROSHandler.ROSHandler(rospy)
    rospy.loginfo("ROSHandler initialized.")

    # Create instance of listener
    listener = hmi_listener.HMICommandListener(rospy, handler)

    # Subscribe using the listener's method
    handler.create_subscriber('hmi/user_command', ControlCommand, listener.command_callback)

    handler.create_publisher('hmi/system_command', ControlCommand)
    handler.create_publisher('light_indication/constant/on', std_msgs.msg.String)
    handler.create_publisher('light_indication/blink/on', std_msgs.msg.String)
    handler.create_publisher('light_indication/off', std_msgs.msg.String)

    rospy.spin()

