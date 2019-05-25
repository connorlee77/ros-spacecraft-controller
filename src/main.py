#!/usr/bin/env python

import numpy as np
import os, rospkg, rospy 

from Controller import Controller
from Constants import Constants
from csv_read import getTrajectoryFromCSV

def main():

    c = Constants()
    
    rp = rospkg.RosPack()
    script_path = os.path.join(rp.get_path("spacecraft_controller"), "src/trajectories", "Positions.csv")
    
    trajectory = getTrajectoryFromCSV(script_path)
    ctrl = Controller(trajectory, Kp=c.KP_DEFAULT, Kd=c.KD_DEFAULT)

if __name__ == '__main__':
    main()
