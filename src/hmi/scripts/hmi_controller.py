#!/usr/bin/env python3

# Import libraries
import os
import signal
import rospkg
import rospy
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk

from hmi.msg import ControlCommand  # Import the custom message
import css_handler
import ros_handler

prefix = "[HMI] "

# Create handler class to manage GUI interactions
class Handler:
    # Initialize the handler with necessary attributes
    def __init__(self):
        self.builder = None
        self.ros_handler = None
        
        self.green_indicator = None
        self.orange_indicator = None
        self.red_indicator = None
        self.cycle_indicator = None
        self.all_indicators = None

        self.start_button_indication = None
        self.stop_button_indication = None
        self.estop_button_indication = None
        self.reset_button_indication = None

        self.estop_active = False

    def start_button(self, button):
        if self.estop_active:
            print("Start blocked: E-STOP is active")
            return

        # Disable the start button
        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(True)
        self.builder.get_object("mode_switch").set_sensitive(False)
        self.builder.get_object("reset_button").set_sensitive(False)

        self.green_indicator.off()
        self.orange_indicator.on()

        self.stop_button_indication.off()
        self.start_button_indication.on()

        self.ros_handler.publish_control_command("START")

    def stop_button(self, button):
        if self.estop_active:
            print("Stop blocked: E-STOP is active")
            return

        # Disable the stop button
        self.builder.get_object("stop_button").set_sensitive(False)
        # Enable start button in case user wants to start again after stop
        self.builder.get_object("start_button").set_sensitive(True)
        self.builder.get_object("mode_switch").set_sensitive(True)
        self.builder.get_object("reset_button").set_sensitive(True)

        self.green_indicator.off()
        self.orange_indicator.blink()

        self.start_button_indication.off()
        self.stop_button_indication.on()

        self.ros_handler.publish_control_command("STOP")

    def estop_button(self, button):
        self.estop_active = True

        # Disable other buttons
        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(False)
        self.builder.get_object("mode_switch").set_sensitive(False)
        self.builder.get_object("reset_button").set_sensitive(True)

        self.all_indicators.off()
        self.red_indicator.on()

        self.start_button_indication.off()
        self.stop_button_indication.off()
        self.estop_button_indication.on()
        self.reset_button_indication.on()

        self.ros_handler.publish_control_command("EMERGENCY_STOP")


    def reset_button(self, button):
        self.estop_active = False

        self.builder.get_object("start_button").set_sensitive(True)
        self.builder.get_object("stop_button").set_sensitive(False)
        self.builder.get_object("mode_switch").set_sensitive(True)
        self.builder.get_object("reset_button").set_sensitive(False)

        self.estop_button_indication.off()
        self.reset_button_indication.off()
        self.start_button_indication.off()
        self.stop_button_indication.off()

        self.all_indicators.off()
        self.green_indicator.on()

        self.ros_handler.publish_control_command("RESET")

    def mode_switch(self, switch, param):
        if switch.get_active():
            self.ros_handler.publish_control_command("CONSTANT")
            self.cycle_indicator.on()
        else:
            self.ros_handler.publish_control_command("SINGLE")
            self.cycle_indicator.off()

class Main:
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('hmi')

        self.builder = Gtk.Builder()
        glade_path = os.path.join(pkg_path, 'resource', 'robot-manager-panel.glade')
        self.builder.add_from_file(glade_path)

        handler = Handler()
        handler.builder = self.builder
        handler.ros_handler = ros_handler.ROSHandler()
        self.handler = handler

        handler.green_indicator = css_handler.GreenIndicator(self.builder)
        handler.orange_indicator = css_handler.OrangeIndicator(self.builder)
        handler.red_indicator = css_handler.RedIndicator(self.builder)
        handler.cycle_indicator = css_handler.CycleIndicator(self.builder)
        handler.all_indicators = css_handler.AllIndicators(self.builder)

        handler.start_button_indication = css_handler.StartButton(self.builder)
        handler.stop_button_indication = css_handler.StopButton(self.builder)
        handler.estop_button_indication = css_handler.EStopButton(self.builder)
        handler.reset_button_indication = css_handler.ResetButton(self.builder)

        self.builder.connect_signals(handler)

        switch = self.builder.get_object("mode_switch")
        switch.connect("notify::active", handler.mode_switch)

        self.css_provider = Gtk.CssProvider()
        css_path = os.path.join(pkg_path, 'resource', 'style.css')
        self.css_provider.load_from_path(css_path)

        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(),
            self.css_provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(False)
        self.builder.get_object("mode_switch").set_sensitive(False)
        handler.reset_button_indication.on()

        self.window = self.builder.get_object("sorting-manager")
        self.window.connect("delete-event", self.on_window_close)
        self.window.show_all()

        handler.ros_handler.add_listener(self.on_ros_message)

    def on_ros_message(self, msg):
        # Example response to received message
        print(prefix + f"[Callback] Received message from ROS topic: {msg.command}")

        if msg.command == "RESET":
            # Optionally trigger UI reset or feedback
            self.builder.get_object("start_button").set_sensitive(True)
            self.builder.get_object("stop_button").set_sensitive(False)
            self.handler.stop_button_indication.off()
            self.handler.orange_indicator.off()
            self.handler.green_indicator.on()



    def on_window_close(self, *args):
        print(prefix + "[INFO] Closing GUI and shutting down ROS...")
        rospy.signal_shutdown("GUI closed by user")
        Gtk.main_quit()

        try:
            ppid = os.getppid()
            print(prefix + f"[INFO] Killing parent roslaunch process (PID: {ppid})")
            os.kill(ppid, signal.SIGINT)
        except Exception as e:
            print(prefix + f"[WARN] Could not kill parent process: {e}")

if __name__ == '__main__':
    print(prefix + "[INFO] Starting Robot Manager Panel\n")
    print(prefix + "[INFO] Current working directory:", os.getcwd())

    rospy.init_node('hmi_controller', anonymous=True)

    main = Main()
    Gtk.main()

    print(prefix + "[INFO] GTK main loop exited")
