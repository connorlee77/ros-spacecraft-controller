#!/usr/bin/env python3

import json
import thruster
import sys
import rospy
from pospos.msg import Thrusters8

thruster_port = sys.argv[1]
thrusters = thruster.ThrusterCtrl(json.load(open('src/pospos/scripts/thrusters.json')), thruster_port)
global cmd_vals 
cmd_vals = {"FX+MZ+":0, "FX+MZ-":0,"FX-MZ+":0,"FX-MZ-":0,"FY+MZ+":0,"FY+MZ-":0,"FY-MZ+":0,"FY-MZ-":0}
global first 
first = False

def cmd_callback(data):
    global first
    first = True
    cmd_vals["FX+MZ+"] = data.FXpMZp
    cmd_vals["FX+MZ-"] = data.FXpMZm
    cmd_vals["FX-MZ+"] = data.FXmMZp
    cmd_vals["FX-MZ-"] = data.FXmMZm
    cmd_vals["FY+MZ+"] = data.FYpMZp
    cmd_vals["FY+MZ-"] = data.FYpMZm
    cmd_vals["FY-MZ+"] = data.FYmMZp
    cmd_vals["FY-MZ-"] = data.FYmMZm

if __name__=='__main__':
    rospy.init_node("server",anonymous=True)
    sub = rospy.Subscriber("thruster_msg",Thrusters8,cmd_callback)
    rate = rospy.Rate(1)

    # if 'fire' in file:
    #     for thruster, duration_ms in file['fire'].items():
    #         print(thruster, duration_ms)
    #         thrusters.fire_pulse(thruster, duration_ms)
    # if 'pwm' in file:
    
    while not rospy.is_shutdown():
        if first:
            for thruster, pulse_width in cmd_vals.items():
                print(thruster, pulse_width)
                thrusters.fire_pulse(thruster, pulse_width)
                # thrusters.set_pwm(thruster, pulse_width)

        rate.sleep()
