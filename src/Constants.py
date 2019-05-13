#!/usr/bin/env python
class Constants:

	def __init__(self):
		self.NAV_RATE = 10;
		self.GUIDE_RATE = 1;

		self.SCSIM_ATT_ERROR = 0.2;
		self.SCSIM_POS_ERROR = 0.2;
		self.SCSIM_ATT_RATE = 1;
		self.SCSIM_POS_RATE = 1;

		self.TEST_ERROR = 0.05;
		self.TEST_RATE = 2;

		self.SMALL_ERROR = 0.3;
		self.SMALL_VELOCITY = 0.01;
		self.BIG_VELOCITY = 0.01;
		self.KP_DEFAULT = 1.5;
		self.KD_DEFAULT = 4;
		self.KP_BIG = 10;
		self.KD_BIG = 10;
