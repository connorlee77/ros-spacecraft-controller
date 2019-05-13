import numpy as np
import roslib
import rospy
#!/usr/bin/env python
import Constants

from spacecraft_controller.msg import Thrusters8 as Thrusters8
from geometry_msgs.msg import TransformStamped as TransformStamped

class Controller:

    # Controller gains
    # Kp - 3x3 matrix
    # Kd - 3x3 matrix
    def __init__(self, Kp=np.eye(3), Kd=np.eye(3)):
        self.Kp = Kp
        self.Kd = Kd
        self.newControl = Thrusters8
        self.prevX = np.zeros((1,3))
        self.elapsedtime = 0

        # initialize
        rospy.init_node('spacecraftcontroller', anonymous=False)
        
        # Current state subscriber
        self.subNavdata = rospy.Subscriber('/vicon/'+controller_name + '/' + controller_name, TransformStamped, self.u)
        # Controls input publisher
        self.publisher = rospy.Publisher('', Thrusters8, queue_size=1)
        # Wait until we receive messages to process in the callback functions
        rospy.spin()

    # data[xd, xdotd, x, xdot] 3x1 vectors
    # output - 3x1
    def u(self, data):

        dt = 0.1
        x = np.array([data.transform.translation.x, 
            data.transform.translation.y,
            data.transform.translation.z])  
        xdot = (self.x - self.prevX) / dt
        
        xrot = np.array([data.transform.rotation.x, 
            data.transform.rotation.y,
            data.transform.rotation.z,
            data.transform.rotation.w])  

        # TODO: desired trajectory
        xd = self.trajectory['xd'][self.elapsedtime]
        xdotd = self.trajectory['xdotd'][self.elapsedtime]

        self.newControl = -self.Kd*(xdot - xdotd) - self.Kp*(x - xd)
        self.publisher.publish(self.newControl)

        self.prevX = x
        rospy.sleep(dt)

def test():
    ctrl = Controller(np.eye(3), np.eye(3))
    print(ctrl.u(np.ones((3,1)), np.ones((3,1)), np.ones((3,1)), np.ones((3,1))))

if __name__ == "__main__":
    test()
