#!/usr/bin/env python

from std_msgs.msg import String
from newspirit.msg import SteerDrive
import rospy
import math
import json

pub = rospy.Publisher("/newspirit/cmd/steerDrive", SteerDrive, queue_size=10)

def get_rover_control(angle, strength):
    """Returns steering and velocity for the rover, to be sent to the Arduino.

    Args:
       angle: Rover steering angle
       strength: Magnitude of the movement

    Returns:
       (steer, drive)

    steering and velocity is between 0 and 1023.
    """
    angle = angle * math.pi / 180
    steer = round(strength * math.cos(angle) / 100 * 511 + 511)
    drive = round(strength * math.sin(angle) / 100 * 511 + 511)

    return (steer, drive)

def sub_callback(data):
    payload = json.loads(data.data)
    angle = payload["angle"]
    strength = payload["strength"]
    (steer, drive) = get_rover_control(angle, strength)
    pub.publish(SteerDrive(steer, drive))
    rospy.loginfo((steer,drive))

if __name__ == "__main__":
    try:
        rospy.init_node('rpi_control', anonymous=True)
        sub = rospy.Subscriber("/newspirit/remote", String, sub_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
