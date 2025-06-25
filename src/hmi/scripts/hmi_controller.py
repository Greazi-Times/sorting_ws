#!/usr/bin/env python3

# Import libraries
import os
import signal
import rospkg
import rospy
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib
import sys

from hmi.msg import ControlCommand  # Import the custom message
import css_handler
import ros_handler
from console_handler import ConsoleHandler
from ros_handler import ROSHandler, ROSLoggerListener

prefix = "[HMI] "

class indication:

    def __init__(self, builder):
        self.builder = builder

        self.green_indicator = css_handler.GreenIndicator(self.builder)
        self.orange_indicator = css_handler.OrangeIndicator(self.builder)
        self.red_indicator = css_handler.RedIndicator(self.builder)
        self.cycle_indicator = css_handler.CycleIndicator(self.builder)
        self.all_indicators = css_handler.AllIndicators(self.builder)

        self.start_button_indication = css_handler.StartButton(self.builder)
        self.stop_button_indication = css_handler.StopButton(self.builder)
        self.estop_button_indication = css_handler.EStopButton(self.builder)
        self.reset_button_indication = css_handler.ResetButton(self.builder)

    def start(self):
        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(True)
        self.builder.get_object("mode_switch").set_sensitive(False)
        self.builder.get_object("reset_button").set_sensitive(False)

        self.green_indicator.off()
        self.orange_indicator.on()

        self.stop_button_indication.off()
        self.start_button_indication.on()

    def stop(self):
        self.builder.get_object("stop_button").set_sensitive(False)
        self.builder.get_object("start_button").set_sensitive(True)
        self.builder.get_object("mode_switch").set_sensitive(True)
        self.builder.get_object("reset_button").set_sensitive(True)

        self.green_indicator.off()
        self.orange_indicator.blink()

        self.start_button_indication.off()
        self.stop_button_indication.on()

    def reset(self):
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

    def estop(self):
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

    def error(self):
        self.builder.get_object("start_button").set_sensitive(False)
        self.builder.get_object("stop_button").set_sensitive(False)
        self.builder.get_object("mode_switch").set_sensitive(False)
        self.builder.get_object("reset_button").set_sensitive(True)

        self.all_indicators.off()
        self.red_indicator.blink()

        self.start_button_indication.off()
        self.stop_button_indication.off()
        self.reset_button_indication.on()

    def cycle(self, on):
        if on:
            self.cycle_indicator.on()
        else:
            self.cycle_indicator.off()


class Handler:
    def __init__(self):
        self.builder = None
        self.ros_handler = None
        self.indication = None
        self.estop_active = False

    def start_button(self, button):
        if self.estop_active:
            print("Start blocked: E-STOP is active")
            return
        self.ros_handler.publish_control_command("START")

    def stop_button(self, button):
        if self.estop_active:
            print("Stop blocked: E-STOP is active")
            return
        self.ros_handler.publish_control_command("STOP")

    def estop_button(self, button):
        self.estop_active = True
        self.ros_handler.publish_control_command("EMERGENCY_STOP")

    def reset_button(self, button):
        self.estop_active = False
        self.ros_handler.publish_control_command("RESET")

    def mode_switch(self, switch, param):
        if switch.get_active():
            self.ros_handler.publish_control_command("CONSTANT")
        else:
            self.ros_handler.publish_control_command("SINGLE")


class Main:

    def on_ros_log(self, msg):
        level_map = {
            1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"
        }
        level = level_map.get(msg.level, "INFO")
        log_line = f"[{level}] [{msg.name}] {msg.msg}\n"
        self.output_redirect.write(log_line)

    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('hmi')

        self.builder = Gtk.Builder()
        glade_path = os.path.join(pkg_path, 'resource', 'robot-manager-panel.glade')
        self.builder.add_from_file(glade_path)

        self.indication = indication(self.builder)

        handler = Handler()
        handler.builder = self.builder
        handler.indication = self.indication
        handler.ros_handler = ros_handler.ROSHandler()
        self.handler = handler

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

        # Run initial error indication safely after show_all
        GLib.idle_add(self.indication.error)

        handler.ros_handler.add_listener(self.on_ros_message)

        textview = self.builder.get_object("console_output")
        output_redirect = ConsoleHandler(textview)

        self.output_redirect = output_redirect
        self.ros_logger_listener = ros_handler.ROSLoggerListener(self.on_ros_log)

    def on_ros_message(self, msg):
        print(prefix + f"[Callback] Received message from ROS topic: {msg.command}")

        if msg.command == "IN-PROGRESS":
            GLib.idle_add(self.indication.start)
        elif msg.command == "STOPPING":
            GLib.idle_add(self.indication.stop)
        elif msg.command == "EMERGENCY_STOP":
            GLib.idle_add(self.indication.estop)
        elif msg.command == "ERROR":
            GLib.idle_add(self.indication.error)
        elif msg.command == "RESET":
            GLib.idle_add(self.indication.reset)
        elif msg.command == "CONSTANT":
            GLib.idle_add(self.indication.cycle, True)
        elif msg.command == "SINGLE":
            GLib.idle_add(self.indication.cycle, False)

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
