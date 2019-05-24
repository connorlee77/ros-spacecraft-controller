#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from Controller import Controller
from Constants import Constants
from csv_read import getTrajectoryFromCSV


def main():

    c = Constants()
    trajectory = getTrajectoryFromCSV('Positions.csv')
    ctrl = Controller(trajectory, Kp=c.KP_DEFAULT, Kd=c.KD_DEFAULT, ROS=False)

if __name__ == '__main__':
	main()