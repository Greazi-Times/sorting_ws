#!/usr/bin/env python
import rospy
from transportband_handler import Transportband
import sys
import termios
import tty

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('toets_control_node')
    band = Transportband()

    print("\nTransportband besturing via toetsen:")
    print(" [t]  Start normaal transport")
    print(" [f]  Foutief object afvoeren")
    print(" [e]  Noodstop activeren")
    print(" [q]  Stoppen\n")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 't':
            rospy.loginfo(">> Start transport via toets [t]")
            band.perform()
        elif key == 'f':
            rospy.loginfo(">> Foutief object afvoeren via toets [f]")
            band.foutiefObject()
        elif key == 'e':
            rospy.logwarn(">> Noodstop geactiveerd via toets [e]")
            band.noodstop()
        elif key == 'q':
            rospy.loginfo(">> Afsluiten...")
            break

if __name__ == "__main__":
    main()

