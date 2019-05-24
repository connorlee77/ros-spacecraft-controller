#!/usr/bin/env python
import numpy as np
import Constants

import roslib
import rospy
from spacecraft_controller.msg import Thrusters8 as Thrusters8
from geometry_msgs.msg import TransformStamped as TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Controller:

    # Controller gains
    # Kp - 3x3 matrix
    # Kd - 3x3 matrix
    def __init__(self, trajectory, Kp=np.eye(3), Kd=np.eye(3), ROS=True):
        
        self.ROS = ROS
        
        self.Kp = np.array(Kp)
        self.Kd = np.array(Kd)
        self.m = 1
        self.I = 1
        self.dt = 0.1
        
        self.control = Thrusters8()
        
        self.prevX = np.zeros((3,1))
        self.trajectory = trajectory
        
        L = 0.2
        b = 0.2
        self.H = np.array([
            [-1, -1, 0, 0, 1, 1, 0, 0], 
            [0, 0, -1, -1, 0, 0, 1, 1],
            [-L, L, -b, b, -L, L, -b, b]])
        
        if self.ROS:
            ### ROS ###
            # initialize
            rospy.init_node('spacecraft_controller', anonymous=False)
            self.time = rospy.Time()
            self.starttime = self.time.now()
            # Current state subscriber
            self.subNavdata = rospy.Subscriber('/vicon/sc3_105/sc3_105', TransformStamped, self.u)
            # Controls input publisher
            self.publisher = rospy.Publisher('Thrusters8', Thrusters8, queue_size=1)
            # Wait until we receive messages to process in the callback functions
            rospy.spin()

    # data[xd, xdotd, x, xdot] 3x1 vectors
    # output - 3x1
    def u(self, data):
        x = np.array([data.transform.translation.x, 
            data.transform.translation.y,
            data.transform.translation.z]).reshape((3,1))  
        xdot = (x - self.prevX) / self.dt
        
        xrot = np.array([data.transform.rotation.x, 
            data.transform.rotation.y,
            data.transform.rotation.z,
            data.transform.rotation.w]) 
        
        roll, pitch, yaw = euler_from_quaternion(xrot)
        
        # TODO: desired trajectory
        if self.ROS:
            current_time = (self.time.now() - self.starttime).to_sec()
            index = current_time / 10
            xd = self.trajectory['xd'][:,index]
            xdotd = self.trajectory['xdotd'][:,index]
        else:
            xd = self.trajectory['xd']
            xdotd = self.trajectory['xdotd']

        heading = yaw
        u = -self.Kd.dot(xdot - xdotd) - self.Kp.dot(x - xd)
        F = np.linalg.pinv(self.H).dot(np.linalg.inv(self.T(heading))).dot(u);

        self.updateThruster8(7.863*F/(1.0/self.dt)-0.009727)
        self.prevX = x
        
        if self.ROS:
            self.publisher.publish(self.control)
            rospy.sleep(self.dt)
        
    def T(self, theta):
        return np.array([
            [np.cos(theta)/self.m, np.sin(theta)/self.m, 0],
            [-np.sin(theta)/self.m, np.cos(theta)/self.m, 0],
            [0, 0, 1/(2.0*self.I)]]);
    
    def updateThruster8(self, F):
        self.control.FXpMZp = F[0]
        self.control.FXpMZm = F[1]
        self.control.FXmMZp = F[2]
        self.control.FXmMZm = F[3]
        self.control.FYpMZp = F[4]
        self.control.FYpMZm = F[5]
        self.control.FYmMZp = F[6]
        self.control.FYmMZm = F[7]
        
        
def test():
    trajectory = {
        'xd':np.zeros((3,1)),
        'xdotd':np.zeros((3,1))
    }
    ctrl = Controller(trajectory, ROS=False)
    data = TransformStamped()
    data.transform.translation.x = 1
    data.transform.translation.y = 1
    data.transform.translation.z = 1
    data.transform.rotation.x = 1
    data.transform.rotation.y = 1
    data.transform.rotation.z = 1
    data.transform.rotation.w = 1
    ctrl.u(data)

if __name__ == "__main__":
    test()
