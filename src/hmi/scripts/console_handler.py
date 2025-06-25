from gi.repository import Gtk, GLib

class ConsoleHandler:
    def __init__(self, textview):
        self.textview = textview
        self.buffer = textview.get_buffer()

    def write(self, message, level=None):
        if message.strip():
            GLib.idle_add(self._append_text, message)

    def _append_text(self, message):
        end_iter = self.buffer.get_end_iter()
        self.buffer.insert(end_iter, message)

        mark = self.buffer.create_mark(None, self.buffer.get_end_iter(), True)
        self.textview.scroll_to_mark(mark, 0.0, True, 0.0, 1.0)
        return False

    def flush(self):
        pass
