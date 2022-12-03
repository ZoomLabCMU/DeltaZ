import numpy as np
from math import *
from serial import Serial
import time


class Delta():
    def __init__(self):
        self.e = 25
        self.f = 50
        self.re = 60
        self.rf = 30
        self.rMax = 30
        self.zMin = -75
        self.zMax = -35
        self.t1, self.t2, self.t3 = 0, 0, 0
        self.x0, self.x1, self.x2 = 0, 0, 0

        # constants
        self.sin120 = np.sqrt(3)/2.0
        self.cos120 = -0.5
        self.tan60 = np.sqrt(3)
        self.sin30 = 0.5
        self.tan30 = 1 / np.sqrt(3)

        # serial communication
        # self.arduino = arduino

    def go_home(self):
        # self.go_to_angle(0, 0, 0)
        self.go_to(0,0,z_safe)
        self.report_angles()

    def go_to(self, x, y, z):
        if self.test_in_workspace(x,y,z):
            self.x0, self.y0, self.z0 = x, y, z
            self.delta_calc_inverse()
            obs_res = self.arduino_write(f"{-self.t1 + 90},{-self.t2 + 90},{-self.t3 + 90}")
            # self.arduino_write(f"")
            # self.arduino_write(f"")
            return obs_res
        else: 
            print(f"{x}, {y}, {z} Are Not In Workspace")
            return False
        self.report_angles()
            

    def report_angles(self):
        pass
        # print(f"AT ANGLE {np.round(self.t1)} {np.round(self.t2)} {np.round(self.t3)}")

    def report_position(self):
        print(f'AT POS {np.round(self.x0)} {np.round(self.y0)} {np.round(self.z0)}')

    def go_to_angle(self, angle_1, angle_2, angle_3):
        x_old, y_old, z_old = self.x0, self.x1, self.x2
        # Set x,y,z from new angles
        self.delta_calc_forward(angle_1, angle_2, angle_3)
        if self.test_in_workspace(self.x0,self.y0,self.z0):
            self.t1, self.t2, self.t3 = angle_1, angle_2, angle_3
            self.arduino_write(f"{int(-self.t1 + 90)},{int(-self.t2 + 90)},{int(-self.t3 + 90)}")
            # self.arduino_write(f"2,{int(-self.t2 + 90)}")
            # self.arduino_write(f"3,{int(-self.t3 + 90)}")
        else:
            print("Not In Workspace")
            self.x0, self.y0, self.z0 = x_old, y_old, z_old
        
        self.report_position()

    def arduino_write(self, command):
        # print(f"GIVEN INPUT COMMAND IS: {command}")
        # print(f"ARDUINO COMMAND: {command}")
        cmd = bytes(command, 'utf-8')
        arduino.write(cmd)   # write position to serial port
        # time.sleep(0.05)ser.readline().split('\r')[0]
        reachedPos = str(arduino.readline()).split('\\')[0].split("'")[1]
        # print(reachedPos.split('\\')[0].split("'")[1])
        return reachedPos # read serial port for arduino echo
        
    def test_in_workspace(self, x, y, z):
        # Are the coodinates in the woirkspace??
        r = np.sqrt(x**2+ y**2)
        if (r<=self.rMax)and(z<=self.zMax)and(z>=self.zMin): return True
        else: return False

    def delta_calc_angle_yz(self, x0, y0, z0, theta):
        y1 = -0.5 * 0.57735 * self.f; # f/2 * tg 30
        #y1 = yy1;
        y0 -= 0.5 * 0.57735 * e;    # shift center to edge
        # z = a + b*y
        a = (x0 * x0 + y0 * y0 + z0 * z0 + self.rf * self.rf - self.re * self.re - y1 * y1) / (2 * z0);
        b = (y1 - y0) / z0;
        # discriminant
        d = -(a + b * y1) * (a + b * y1) + self.rf * (b * b * self.rf + self.rf);
        if (d < 0):
            return -1; # non-existing point
        yj = (y1 - a * b - sqrt(d)) / (b * b + 1); # choosing outer point
        zj = a + b * yj;

        if yj>y1: ang = 180
        else: ang = 0

        theta = 180.0 * np.arctan2(-zj , (y1 - yj)) / np.pi + ang;
        if (theta < -180) or (theta > 180):
            return -1, theta;
        return 0, theta;

    def delta_calc_inverse(self):
        self.t1, self.t2, self.t3 = 0, 0, 0
        stat1, self.t1 = self.delta_calc_angle_yz(self.x0, self.y0, self.z0, self.t1)
        stat2, self.t2 = self.delta_calc_angle_yz(self.x0*self.cos120 + self.y0*self.sin120, self.y0*self.cos120 - self.x0*self.sin120, self.z0, self.t2)
        stat3, self.t3 = self.delta_calc_angle_yz(self.x0*self.cos120 - self.y0*self.sin120, self.y0*self.cos120 + self.x0*self.sin120, self.z0, self.t3)
        return stat1 + stat2 + stat3

    def delta_calc_forward(self, angle_1, angle_2, angle_3):
        # Calculate forward positions given the angle.
        t = (self.f - self.e) * self.tan30 / 2
        dtr = np.pi / 180
        angle_1 *= dtr
        angle_2 *= dtr
        angle_3 *= dtr

        y1 = -(t + self.rf * cos(angle_1))
        z1 = -self.rf * sin(angle_1)

        y2 = (t + self.rf * cos(angle_2)) * self.sin30
        x2 = y2 * self.tan60
        z2 = -self.rf * sin(angle_2)

        y3 = (t + self.rf * cos(angle_3)) * self.sin30
        x3 = -y3 * self.tan60
        z3 = -self.rf * sin(angle_3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - self.re * self.re)

        # discriminant
        d = b * b - (4.0 * a * c)
        if (d < 0):
            return -1  # non-existing point

        self.z0 = -(0.5 * (b + sqrt(d)) / a)
        self.x0 = (a1 * self.z0 + b1) / dnm
        self.y0 = (a2 * self.z0 + b2) / dnm
        return 0




arduino = Serial('/dev/ttyUSB0', baudrate=9600) 
time.sleep(2)
if __name__=="__main__":
    z_safe = -40
    z_move = -51
    delta = Delta()
    # time.sleep(2)
    delta.go_home()

    """ TEST """

    # for i in range(-30,-50, -1):
    #     delta.go_to(-7.49,5.52, i)

    # for i in [
    #     [-6.49,-2,z_safe],[-6.49,-2,z_move],[-6.49,-2,z_safe],
    #     [-6.49,6.65,z_safe],[-6.49,6.65,z_move],
    #     [2.25, 5.9, z_safe],[2.25, 5.9, z_move],[2.25, 5.9, z_safe],
    #     [-4.75, 7, z_safe],[-4.75, 7, z_move]
    #     ]:
    #     delta.go_to(*i)
        # time.sleep(100)
    while True:   
        """ TURN POT CW """
        x0, y0 = -6.49,6.65
        theta_cw = np.arctan2(y0,x0)
        x1, y1 = 2.25, 5.9
        theta_ccw = np.arctan2(y1,x1)
        thetas = list(np.arange(theta_cw, np.pi*2, np.pi/8))+ list(np.arange(0, theta_ccw,np.pi/8))[1:]
        r = 10
        delta.go_to(-6.49,6.65,z_safe)

        for theta in thetas:
            x = r*np.cos(theta)
            y = r*np.sin(theta)
            delta.go_to(x, y, z_move)

        """ TURN POT CCW """
        x0, y0 = 2.25, 5.9
        theta_cw = np.arctan2(y0,x0)
        x2, y2 = -6.49,-2
        theta_ccw = np.arctan2(y0,x0)
        # thetas = list(np.arange(theta_cw, np.pi*2, np.pi/15))+ list(np.arange(0, theta_ccw,np.pi/15))[1:]
        r = 10
        # delta.go_to(2.25, 5.9,z_safe)
        # delta.go_to(-4.75, 7,z_safe)
        thetas_invert = list(np.arange(theta_cw, np.pi*2, np.pi/4))+ list(np.arange(0, theta_cw, np.pi/4))[1:]
        thetas = list(np.arange(theta_cw, np.pi*2, np.pi/10))+ list(np.arange(0, theta_ccw,np.pi/10))[1:-2]
        
        for theta in thetas_invert[::-1]:
            x = r*np.cos(theta)
            y = r*np.sin(theta)
            delta.go_to(x, y, z_move)

        for theta in thetas[::-1]:
            x = r*np.cos(theta)
            y = r*np.sin(theta)
            delta.go_to(x, y, z_move)

        delta.go_to(-6.49,-2,z_safe)
        delta.go_to(-6.49,6.65,z_safe)







        # """ USER INPUT """   
        # command = input ("Servo position: ")
        # x,y,z = np.array(command.split(','), dtype=float)
        # delta.go_to(x, y, z)
        # """ FULL CIRCLE """
        # resolution = 10   

        # theta = np.linspace(0, 2*np.pi, resolution)
        # r = 20
        # x = r*np.cos(theta)
        # y = r*np.sin(theta)
        # z = -50

        # for i in range(resolution):
        #     delta.go_to(x[i], y[i], z)
        #     # time.sleep(1)
        # else:
        #     delta.go_home()
        #     break