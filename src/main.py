#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from Controller import Controller
from Constants import Constants

def main():

	c = Constants()
	ctrl = Controller(Kp=c.KP_DEFAULT, Kd=c.KD_DEFAULT)

if __name__ == '__main__':
	main()