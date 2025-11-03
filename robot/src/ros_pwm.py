#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import Adafruit_PCA9685
from math import pi

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output


PWM = Adafruit_PCA9685.PCA9685(address=0x40, busnum=0)
PWM.set_pwm_freq(60)
on = 0

pid_w1 = PIDController(0.5, 0.01, 0.05)
pid_w2 = PIDController(0.5, 0.01, 0.05)
pid_w3 = PIDController(0.5, 0.01, 0.05)
pid_w4 = PIDController(0.5, 0.01, 0.05)


def WFL1(LF, LB):
    PWM.set_pwm(12, on, int(LF))
    PWM.set_pwm(13, on, int(LB))

def WFR1(LF, LB):
    PWM.set_pwm(14, on, int(LF))
    PWM.set_pwm(15, on, int(LB))

def WFL2(LF, LB):
    PWM.set_pwm(10, on, int(LF))
    PWM.set_pwm(11, on, int(LB))

def WFR2(LF, LB):
    PWM.set_pwm(8, on, int(LF))
    PWM.set_pwm(9, on, int(LB))



def vel_callback(msg):
    max_pwm = 4095
    wheel_r = 0.04
    Lx = 0.25
    Ly = 0.24

    # inverse kinematics
    wf1_target = (1/wheel_r)*((msg.linear.x-msg.linear.y)-(Lx+Ly)*msg.angular.z)
    wf2_target = (1/wheel_r)*((msg.linear.x+msg.linear.y)+(Lx+Ly)*msg.angular.z)
    wf3_target = (1/wheel_r)*((msg.linear.x+msg.linear.y)-(Lx+Ly)*msg.angular.z)
    wf4_target = (1/wheel_r)*((msg.linear.x-msg.linear.y)+(Lx+Ly)*msg.angular.z)

    w1, w2, w3, w4 = get_encoder_speed()

    pwm1 = pid_w1.compute(wf1_target, w1)
    pwm2 = pid_w2.compute(wf2_target, w2)
    pwm3 = pid_w3.compute(wf3_target, w3)
    pwm4 = pid_w4.compute(wf4_target, w4)

    def output_motor(pwm, WFL, reverse):
        pwm = int(max(min(abs(pwm), max_pwm), 0))
        if reverse:
            WFL(0, pwm)
        else:
            WFL(pwm, 0)

    output_motor(pwm1, WFL1, pwm1 < 0)
    output_motor(pwm2, WFR1, pwm2 < 0)
    output_motor(pwm3, WFL2, pwm3 < 0)
    output_motor(pwm4, WFR2, pwm4 < 0)


if __name__ =='__main__':
    rospy.init_node('mecanum_pid_node')
    sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
    rospy.spin()
