#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from Controller import Controller
from Constants import Constants

def main():

    c = Constants()
    trajectory = {
        'xd':np.zeros((3,1)),
        'xdotd':np.zeros((3,1))
    }
    ctrl = Controller(trajectory, Kp=c.KP_DEFAULT, Kd=c.KD_DEFAULT)

if __name__ == '__main__':
	main()