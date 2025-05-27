
import gi
from gi.repository import Gtk as gtk

gi.require_version('Gtk', '3.0')

class Main:
    def __init__(self):
        gladeFile = "main.glade"
        self.builder = gtk.Builder()
        self.builder.add_from_file(gladeFile)

        button = self.builder.get_object("button")
        button.connect("clicked", self.printText)

        window = self.builder.get_object("main")
        window.connect("delete-event", gtk.main_quit)
        window.show_all()

    def printText(self, widget):
        print("Hello World!")


if __name__ == '__main__':
    main = Main()
    # The main method is empty because the setup() method is called in the constructor.
    # All the GUI setup and event connections are handled in the setup method.
    # This allows for a clean separation of concerns and makes the code easier to maintain.
    gtk.main()  # Start the GTK main loop