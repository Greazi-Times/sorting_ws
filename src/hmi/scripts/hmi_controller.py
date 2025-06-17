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

# Create handler class to manage GUI interactions
class Handler:
    # Initialize the handler with necessary attributes
    def __init__(self):
        self.builder = None
        self.ros_handler = None
        
        self.green_indicator = None
        self.orange_indicator = None
        self.red_indicator = None
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
        print("Start button")

        # Disable the start button
        self.builder.get_object("start_button").set_sensitive(False)
        # Ensure stop button remains enabled
        self.builder.get_object("stop_button").set_sensitive(True)

        self.green_indicator.off()
        self.orange_indicator.on()

        self.stop_button_indication.off()
        self.start_button_indication.on()

        self.ros_handler.publish_control_command("START")

    def stop_button(self, button):
        if self.estop_active:
            print("Stop blocked: E-STOP is active")
            return
        print("Stop button")

        # Disable the stop button
        self.builder.get_object("stop_button").set_sensitive(False)
        # Enable start button in case user wants to start again after stop
        self.builder.get_object("start_button").set_sensitive(True)

        self.green_indicator.off()
        self.orange_indicator.blink()

        self.start_button_indication.off()
        self.stop_button_indication.on()

        self.ros_handler.publish_control_command("STOP")

    def estop_button(self, button):
        print("E-STOP button")
        self.estop_active = True

        # Disable other buttons
        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(False)

        self.all_indicators.off()
        self.red_indicator.on()

        self.start_button_indication.off()
        self.stop_button_indication.off()
        self.estop_button_indication.on()
        self.reset_button_indication.on()

        self.ros_handler.publish_control_command("EMERGENCY_STOP")


    def reset_button(self, button):
        print("Reset button")
        self.estop_active = False

        self.builder.get_object("start_button").set_sensitive(True)
        self.builder.get_object("stop_button").set_sensitive(False)

        self.estop_button_indication.off()
        self.reset_button_indication.off()
        self.start_button_indication.off()
        self.stop_button_indication.off()

        self.all_indicators.off()
        self.green_indicator.on()

        self.ros_handler.publish_control_command("RESET")

    def mode_switch(self, switch, param):
        print("Mode switch toggled")
        if switch.get_active():
            print("Switch ON")
        else:
            print("Switch OFF")



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

        handler.green_indicator = css_handler.GreenIndicator(self.builder)
        handler.orange_indicator = css_handler.OrangeIndicator(self.builder)
        handler.red_indicator = css_handler.RedIndicator(self.builder)
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

        self.window = self.builder.get_object("sorting-manager")
        self.window.connect("delete-event", self.on_window_close)
        self.window.show_all()



    def on_window_close(self, *args):
        print("[INFO] Closing GUI and shutting down ROS...")
        rospy.signal_shutdown("GUI closed by user")
        Gtk.main_quit()

        try:
            ppid = os.getppid()
            print(f"[INFO] Killing parent roslaunch process (PID: {ppid})")
            os.kill(ppid, signal.SIGINT)
        except Exception as e:
            print(f"[WARN] Could not kill parent process: {e}")

if __name__ == '__main__':
    print("[INFO] Starting Robot Manager Panel\n")
    print("[INFO] Current working directory:", os.getcwd())

    rospy.init_node('hmi_controller', anonymous=True)

    main = Main()
    Gtk.main()

    print("[INFO] GTK main loop exited")
