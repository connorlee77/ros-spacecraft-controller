import time
import serial



class ThrusterCtrl:
    def __init__(self, cfg, port):
        self.fd = serial.Serial(port, 115200)
        self.cfg = cfg
        time.sleep(2) # wait for arduino to boot
        self.fd.write(b'\n')

    def __del__(self):
        time.sleep(0.1) # make sure the last command terminates befor closing the port


    def set_pwm(self, thruster, pwm):
        thruster_id = self.cfg['port'][thruster]
        pwm = min(max(pwm, 0), 1) # limit to 0...1
        cmd = "pwm {} {}\n".format(thruster_id, int(pwm*(4096-1)))
        print(cmd.rstrip('\n'))
        self.fd.reset_input_buffer()
        self.fd.write(cmd.encode())
        self.fd.flush()

        # time.sleep(0.01)

    def fire_pulse(self, thruster, duration_ms):
        thruster_id = self.cfg['port'][thruster]
        duration_ms = max(duration_ms, 0)  # make sure it is positive
        cmd = "fire {} {}\n".format(thruster_id, int(duration_ms))
        print(cmd.rstrip('\n'))
        self.fd.reset_input_buffer()
        self.fd.write(cmd.encode())
        self.fd.flush()

if __name__ == '__main__':
    import json
    import sys
    t = ThrusterCtrl(json.load(open('thrusters.json')), sys.argv[1])

    import time
    # while 1:
    #     # print('x')
    #     # t.fire([0.5, 0, 0], [0, 0, 0])
    #     # time.sleep(0.1)
    #     # print('y')
    #     # t.fire([0, 0.5, 0], [0, 0, 0])
    #     # time.sleep(0.1)
    #     print('z')
    #     # t.fire([0, 0, 0.5], [0, 0, 0])
    #     # time.sleep(0.1)
    #     t.fire([0, 0, 1], [0, 0, 0])
    #     time.sleep(0.1)

    #     print('off')
    #     t.fire([0, 0, 0], [0, 0, 0])
    #     time.sleep(0.1)

    t.fire([0.5, 0, 0], [0, 0, 0])

    time.sleep(1)
    t.fire([0, 0, 0], [0, 0, 0])
