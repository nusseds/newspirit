#!/usr/bin/env python

from std_msgs.msg import String
from newspirit.msg import SteerDrive
import rospy
import math
import json
from bluetooth import *

server_sock = BluetoothSocket(RFCOMM)
server_sock.bind(("", PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
advertise_service(server_sock, "PiSrv", service_id=uuid, service_classes=[uuid, SERIAL_PORT_CLASS], profiles=[SERIAL_PORT_PROFILE])

def get_rover_control(angle, strength):
    """Returns steering and velocity for the rover, to be sent to the Arduino.

    Args:
       angle: Rover steering angle
       strength: Magnitude of the movement

    Returns:
       (steer, drive)

    steering and velocity is between 0 and 1023.
    """
    steer = round(strength * math.sin(angle) / 100 * 511 + 511)
    drive = round(strength * math.cos(angle) / 100 * 511 + 511)

    return (steer, drive)


def talker():
    pub = rospy.Publisher("/newspirit/cmd/steerDrive", SteerDrive, queue_size=10)
    rospy.init_node('rpi_control', anonymous=True)
    rate=rospy.Rate(10)
    rospy.loginfo("Waiting for connection on RFCOMM channel %d" % port)
    client_sock, client_info = server_sock.accept()
    rospy.loginfo("Accepted connection from {}".format(client_info))

    while not rospy.is_shutdown():
        try:
            data = client_sock.recv(1024)
            if (len(data) == 0): break

            payload = json.loads(data)
            rc = payload["rover_control"]

            (x, y) = get_rover_control(rc["angle"], rc["strength"])

            pub.publish(SteerDrive(x, y))
            rospy.loginfo((x,y))

        except ValueError:
            # This happens when the bluetooth messages get duplicated
            # Here we ignore them and discard the data
            pass
        except IOError:
            pass

        except KeyboardInterrupt:
            client_sock.close()
            server_sock.close()

        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
