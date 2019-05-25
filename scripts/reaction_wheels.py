#!/usr/bin/env python
import odrive
import odrive.enums
import fibre.protocol
import json
import time
import sys
from hivemind.srv import reaction_wheel

MAX_RPM = 5000
POLE_PAIRS = 8
RESISTANCE_CALIB_MAX_VOLTAGE = 1.0
CALIB_CURRENT = 2.0
CURRENT_LIM = 3.0
REQUESTED_CURRENT_RANGE = 1.0
HALL_CPR = 6 * POLE_PAIRS
POS_GAIN = 1.0
VEL_GAIN = 0.01
VEL_LIMIT = MAX_RPM

rctl = None

class Wheel:
    def __init__(self, drive_sn, axis_no):
        self.drive_sn = drive_sn
        self.axis_no = axis_no


class ReactionWheelCtrl:
    def __init__(self, json_data):
        self.drives = {}
        self.wheels = {}
        for wheel_no, wheel_data in enumerate(json_data["wheels"]):
            new_wheel = Wheel(wheel_data["drive_sn"], wheel_data["axis_no"])
            print("Index: {}, Drive SN: {}, Axis No: {}" \
                    .format(wheel_no, new_wheel.drive_sn, new_wheel.axis_no))
            self.wheels[wheel_no] = new_wheel
            if new_wheel.drive_sn not in self.drives:
                print("Finding Odrive with SN {}".format(new_wheel.drive_sn))
                new_drive = odrive.find_any(serial_number=new_wheel.drive_sn)
                self.drives[new_wheel.drive_sn] = new_drive
        time.sleep(2) # wait for arduino to boot


    def __del__(self):
        time.sleep(0.1) # make sure the last command terminates


    def reboot(self, drive_sn, axis_no=0):
        drv = self.drives[drive_sn]
        try:
            drv.reboot()
        except fibre.protocol.ChannelBrokenException as e:
            pass
        drv = odrive.find_any(serial_number=drive_sn)
        self.drives[drive_sn] = drv
        return (drv, drv.axis0) if axis_no == 0 else (drv, drv.axis1)


    def configure(self, wheel_no):
        drive_sn = self.wheels[wheel_no].drive_sn
        axis_no = self.wheels[wheel_no].axis_no
        print("Configuring drive {} axis {}".format(drive_sn, axis_no))
        drv = self.drives[drive_sn]
        axis = drv.axis0 if axis_no == 0 else drv.axis1
        axis.motor.config.pole_pairs = POLE_PAIRS
        axis.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
        axis.motor.config.calibration_current = CALIB_CURRENT
        axis.motor.config.current_lim = CURRENT_LIM
        axis.motor.config.requested_current_range = REQUESTED_CURRENT_RANGE
        axis.encoder.config.mode = odrive.enums.ENCODER_MODE_HALL
        axis.encoder.config.cpr = HALL_CPR
        axis.controller.config.pos_gain = POS_GAIN
        axis.controller.config.vel_gain = VEL_GAIN
        axis.controller.config.vel_limit = VEL_LIMIT
        axis.controller.config.control_mode = odrive.enums.CTRL_MODE_VELOCITY_CONTROL

        print("Saving motor config")
        drv.save_configuration()
        drv, axis = self.reboot(drive_sn, axis_no)

        print("Calibrating motor")
        axis.requested_state = odrive.enums.AXIS_STATE_MOTOR_CALIBRATION
        while axis.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        if axis.motor.error == 0:
            axis.motor.config.pre_calibrated = True
        else:
            print("Motor error: 0x{}".format(axis.motor.error, '02x'))

        print("Calibrating encoder")
        axis.requested_state = odrive.enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        while axis.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        if axis.encoder.error == 0:
            axis.encoder.config.pre_calibrated = True
        else:
            print("Encoder error: 0x{}".format(axis.encoder.error, '02x'))

        print("Saving encoder config")
        drv.save_configuration()
        drv, axis = self.reboot(drive_sn, axis_no)


    def set_rpm(self, wheel_no, rpm):
        drive = self.drives[self.wheels[wheel_no].drive_sn]
        axis = drive.axis0 if self.wheels[wheel_no].axis_no == 0 else drive.axis1
        rpm = min(max(rpm, 0), MAX_RPM) # limit to min..max
        axis.controller.vel_setpoint = rpm

    def enable(self, wheel_no):
        drive = self.drives[self.wheels[wheel_no].drive_sn]
        axis = drive.axis0 if self.wheels[wheel_no].axis_no == 0 else drive.axis1
        axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

def handle_wheel_request(req):
    if req.wheel_no >3:
        return [False]
    rctl.set_rpm(req.wheel_no, req.rpm)
    return [True]


if __name__ == '__main__':
    print("Initializing")
    global rctl
    rctl = ReactionWheelCtrl(json.load(open('reaction_wheels.json')))
    print("Done initializing")
    print("Running")
    rospy.init_node('reaction_wheel_server')
    s = rospy.Service('reaction_wheel', reaction_wheel, handle_wheel_request)
