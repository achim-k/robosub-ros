#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool

SERVO_SERVICE = 'servo_control'


# Call the servo control service
def call_servo_control(data):
    rospy.wait_for_service(SERVO_SERVICE)
    try:
        servo_control = rospy.ServiceProxy(SERVO_SERVICE, SetBool)
        resp = servo_control(data)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')


if __name__ == '__main__':
    rospy.init_node('servo_control_client')
    # Call the servo control service to move the servo to the left
    # call_servo_control(True)
    # Call the servo control service to move the servo to the right
    call_servo_control(False)

