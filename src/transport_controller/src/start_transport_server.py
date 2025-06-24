#!/usr/bin/env python
import rospy
from transport_controller.srv import StartTransport, StartTransportResponse
from transportband_handler import Transportband

band = None

def handle_transport_request(req):
    rospy.loginfo("StartTransport service ontvangen.")
    success = band.perform()
    return StartTransportResponse(success=success)

def main():
    global band
    rospy.init_node('start_transport_server')
    band = Transportband()
    rospy.Service('/transportband/start', StartTransport, handle_transport_request)
    rospy.loginfo("Service '/transportband/start' actief.")
    rospy.spin()

if __name__ == "__main__":
    main()
