#!/usr/bin/python3
# -*- coding: utf-8 -*-

from gi.repository import Gtk

class Handler:
    def start(self, button):
      print("Start button")

builder = Gtk.Builder()
builder.add_from_file("robot-management.glade")
builder.connect_signals(Handler())

ournewbutton = builder.get_object("start")

window = builder.get_object("robot-management")

window.connect("delete-event", Gtk.main_quit)
window.show_all()
Gtk.main()