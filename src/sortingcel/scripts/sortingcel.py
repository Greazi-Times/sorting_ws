#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import libraries
import rospy
import ROSHandler

# Messages
from hmi.msg import ControlCommand
from hmi_listener import HMICommandListener
from transport_handler import Transportsystem
from camera_handler import CameraHandler
from robot_handler import RobotHandler
from xarm_msgs.msg import RobotMsg
from transfer_util import TransformUtil

import std_msgs
from std_msgs.msg import Bool

state = None

class sorting_system:
    """
    Class which handles the whole project
    """

    def __init__(self, rospy, ros_handler, hmi_handler, transport_handler, camera_handler=None, robot_handler=None):
        """
        Initialize the sorting system.
        """
        self.rospy = rospy
        self.ros_handler = ros_handler
        self.hmi_handler = hmi_handler
        self.transport_handler = transport_handler
        self.camera_handler = camera_handler
        self.robot_handler = robot_handler
        self.transfer_util = TransformUtil()

        self.cycle_mode = False  # Default to single mode
        self.stopping = False  # Flag to indicate if the system is stopping

        rospy.loginfo("Sorting system initialized.")

    def update_state(self, new_state):
        """
        Update the current state of the sorting system.
        """
        global state
        rospy.loginfo("Updating state from {} to {}".format(state, new_state))
        state = new_state
        self.ros_handler.publish('hmi/system_command', ControlCommand(command=new_state))

    def reset_system(self):
        """
        Reset the sorting system to its initial state.
        """
        rospy.loginfo("Resetting sorting system...")
        self.update_state("RESET")
        self.robot_handler.reset_robot()
        rospy.loginfo("Sorting system reset complete.")
        self.hmi_handler.reset_indication()

    def start_system(self):
        """
        Start the sorting system.
        """
        if self.cycle_mode:
            self.hmi_handler.in_progress()
            self.update_state("IN-PROGRESS")
        else:
            self.hmi_handler.in_progress()
            self.update_state("STOPPING")
            self.stopping = True
            rospy.sleep(0.5)
            self.hmi_handler.stop_system()

        if self.transport_handler.perform():
            rospy.loginfo("Transport system has successfully finished its task.")
        else:
            rospy.logerr("Transport system failed to complete its task.")
            self.hmi_handler.error()
            self.update_state("ERROR")
            return

        # Check camera detection
        info = self.camera_handler.check()
        if info:
            print("Detected object: {} at ({}, {}, {}) with angle {}".format(
                info.object_class, info.X, info.Y, info.Z, info.angle))
        else:
            rospy.logerr("No detection info received from camera.")
            self.hmi_handler.error()
            self.update_state("ERROR")
            return

        # Robot perform method
        result = self.transfer_util.transform(info.X, info.Y, info.Z, info.qx, info.qy, info.qz, info.qw)

        if not result:
            rospy.logerr("Could not perform transform.")
            self.hmi_handler.error()
            self.update_state("ERROR")
            return

        x_w, y_w, z_w, orientation = result

        if not self.robot_handler.sort(x_w, y_w, z_w, orientation, info.object_class):
            rospy.loggerr("Could not move the robot, 10/10 failed!")
            self.hmi_handler.error()
            self.update_state("ERROR")
            return

        if self.cycle_mode and not self.stopping:
            rospy.loginfo("Continues mode activated starting loop again.")
            self.start_system()
        else:
            rospy.loginfo("Stopping system.")
            self.reset_system()

    def stop_system(self):
        """
        Stops the system on the end of the cycle.
        """
        self.stopping = True
        self.hmi_handler.stop_system()
        self.update_state("STOPPING")

    def emergency_stop(self):
        """
        Emergency stop action for the sorting system.
        """
        rospy.logerr("Emergency stop activated!")
        self.hmi_handler.emergency_stop()
        self.update_state("EMERGENCY_STOP")
        self.transport_handler.noodstop()

    def cycle(self, on):
        """ 
        Toggle the cycle mode of the sorting system.
        """
        self.cycle_mode = on
        self.update_state("CONSTANT" if on else "SINGLE")
        rospy.loginfo("Cycle mode set to {}.".format("CONSTANT" if on else "SINGLE"))


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
        global state
        rospy.loginfo("[HMI Command] Received command: {}".format(msg.command))

        command_msg = msg.command
        if command_msg == "START":
            self.sorting_system.start_system()
        elif command_msg == "STOP":
            self.sorting_system.stop_system()
        elif command_msg == "RESET":
            self.sorting_system.reset_system()
        elif command_msg == "EMERGENCY_STOP":
            self.sorting_system.emergency_stop()
        elif command_msg == "CONSTANT":
            self.sorting_system.cycle(True)
        elif command_msg == "SINGLE":
            self.sorting_system.cycle(False)
        else:
            rospy.logwarn("[HMI Listener] Unknown command: {}".format(msg.command))
            state = "ERROR"


# Main initialization function
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('sortingcel', anonymous=True)
    rospy.loginfo("Starting sortingcel system...")

    ros_handler = ROSHandler.ROSHandler(rospy)
    rospy.loginfo("ROSHandler initialized.")

    transport_handler = Transportsystem(rospy, ros_handler)
    hmi_handler = HMICommandListener(rospy, ros_handler)
    camera_handler = CameraHandler()
    robot_handler = RobotHandler()

    sorting = sorting_system(rospy, ros_handler, hmi_handler, transport_handler, camera_handler, robot_handler)
    subscribers_node = subscribers(rospy, hmi_handler, sorting)

    # Subscribe
    ros_handler.create_subscriber('hmi/user_command', ControlCommand, subscribers_node.hmi_command_callback)
    ros_handler.create_subscriber('transportsystem/sensor/start', Bool, transport_handler._beginsensor_callback)
    ros_handler.create_subscriber('transportsystem/sensor/end', Bool, transport_handler._eindsensor_callback)
    ros_handler.create_subscriber("/ufactory/robot_states", RobotMsg, robot_handler._robot_state_callback)
    ros_handler.create_subscriber("/custom/emergency_stop", Bool, robot_handler.external_estop_callback)

    # Publishers
    ros_handler.create_publisher('hmi/system_command', ControlCommand)
    ros_handler.create_publisher('light_indication/constant/on', std_msgs.msg.String)
    ros_handler.create_publisher('light_indication/blink/on', std_msgs.msg.String)
    ros_handler.create_publisher('light_indication/off', std_msgs.msg.String)
    ros_handler.create_publisher('transportsystem/command', Bool)

    rospy.spin()
    rospy.loginfo("Sortingcel system is running. Waiting for commands...")
