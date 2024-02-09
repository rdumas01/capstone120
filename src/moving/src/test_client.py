#!/usr/bin/env python3
import rospy
from Test_service.srv import CoordinateService

def coordinate_client(x1, y1, z1, x2, y2, z2, x3, y3, z3):
    rospy.wait_for_service('coordinate_service')
    try:
        coordinate_service = rospy.ServiceProxy('coordinate_service', CoordinateService)
        resp = coordinate_service(x1, y1, z1, x2, y2, z2, x3, y3, z3)
        return resp.confirm
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    # 預先給定的三組座標
    x1, y1, z1 = 1.0, 2.0, 3.0
    x2, y2, z2 = 4.0, 5.0, 6.0
    x3, y3, z3 = 7.0, 8.0, 9.0

    print(f"Requesting coordinates ({x1}, {y1}, {z1}), ({x2}, {y2}, {z2}), ({x3}, {y3}, {z3})")
    print(f"{coordinate_client(x1, y1, z1, x2, y2, z2, x3, y3, z3)}")
