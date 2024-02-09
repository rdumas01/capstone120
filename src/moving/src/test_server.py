#!/usr/bin/env python3
import rospy
from Test_service.srv import CoordinateService, CoordinateServiceResponse

def handle_coordinate_service(req):
    print(f"Received coordinates: ({req.x1}, {req.y1}, {req.z1}), ({req.x2}, {req.y2}, {req.z2}), ({req.x3}, {req.y3}, {req.z3})")
    return CoordinateServiceResponse("Coordinates received successfully")

def coordinate_server():
    rospy.init_node('coordinate_server')
    s = rospy.Service('coordinate_service', CoordinateService, handle_coordinate_service)
    print("Ready to receive coordinates.")
    rospy.spin()

if __name__ == "__main__":
    coordinate_server()
