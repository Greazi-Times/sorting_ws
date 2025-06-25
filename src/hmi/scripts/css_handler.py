#!/usr/bin/env python3

import rospy

class css_handler:
    def __init__(self, builder):
        self.builder = builder

    def get_context(self, widget_id):
        widget = self.builder.get_object(widget_id)
        if widget is None:
            rospy.logerr(f"[Indicator] Widget with ID '{widget_id}' not found.")
            return None
        return widget.get_style_context()

    def add_class(self, widget_id, class_name):
        ctx = self.get_context(widget_id)
        if ctx:
            ctx.add_class(class_name)

    def remove_class(self, widget_id, class_name):
        ctx = self.get_context(widget_id)
        if ctx:
            ctx.remove_class(class_name)


class GreenIndicator(css_handler):
    def on(self):
        self.remove_class("status_indicator_green", "indicator-green-blink")
        self.add_class("status_indicator_green", "indicator-green-on")

    def blink(self):
        self.remove_class("status_indicator_green", "indicator-green-on")
        self.add_class("status_indicator_green", "indicator-green-blink")

    def off(self):
        self.remove_class("status_indicator_green", "indicator-green-on")
        self.remove_class("status_indicator_green", "indicator-green-blink")


class OrangeIndicator(css_handler):
    def on(self):
        self.remove_class("status_indicator_orange", "indicator-orange-blink")
        self.add_class("status_indicator_orange", "indicator-orange-on")

    def blink(self):
        self.remove_class("status_indicator_orange", "indicator-orange-on")
        self.add_class("status_indicator_orange", "indicator-orange-blink")

    def off(self):
        self.remove_class("status_indicator_orange", "indicator-orange-on")
        self.remove_class("status_indicator_orange", "indicator-orange-blink")


class RedIndicator(css_handler):
    def on(self):
        self.remove_class("status_indicator_red", "indicator-red-blink")
        self.add_class("status_indicator_red", "indicator-red-on")

    def blink(self):
        self.remove_class("status_indicator_red", "indicator-red-on")
        self.add_class("status_indicator_red", "indicator-red-blink")

    def off(self):
        self.remove_class("status_indicator_red", "indicator-red-on")
        self.remove_class("status_indicator_red", "indicator-red-blink")

class CycleIndicator(css_handler):
    def on(self):
        self.add_class("status_indicator_cycle", "indicator-cycle-on")

    def off(self):
        self.remove_class("status_indicator_cycle", "indicator-cycle-on")

class AllIndicators(css_handler):
    def off(self):
        self.remove_class("status_indicator_green", "indicator-green-on")
        self.remove_class("status_indicator_green", "indicator-green-blink")
        self.remove_class("status_indicator_orange", "indicator-orange-on")
        self.remove_class("status_indicator_orange", "indicator-orange-blink")
        self.remove_class("status_indicator_red", "indicator-red-on")
        self.remove_class("status_indicator_red", "indicator-red-blink")


class StartButton(css_handler):
    def on(self):
        self.add_class("start_button", "pulse-green")

    def off(self):
        self.remove_class("start_button", "pulse-green")

class StopButton(css_handler):
    def on(self):
        self.add_class("stop_button", "pulse-red")

    def off(self):
        self.remove_class("stop_button", "pulse-red")

class EStopButton(css_handler):
    def on(self):
        self.add_class("estop_button", "pulse-yellow-red")

    def off(self):
        self.remove_class("estop_button", "pulse-yellow-red")

class ResetButton(css_handler):
    def on(self):
        self.add_class("reset_button", "pulse-blue")

    def off(self):
        self.remove_class("reset_button", "pulse-blue")
