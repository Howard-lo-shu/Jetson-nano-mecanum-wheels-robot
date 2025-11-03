#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import smbus
import math
from math import sin, cos, pi
import tf 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#####MOTORSET#####
string=''
import RPi.GPIO as GPIO
import Adafruit_PCA9685
from math import pi,sin,cos
cmd = Twist()
PWM = Adafruit_PCA9685.PCA9685(address=0x40, busnum=0)
PWM.set_pwm_freq(60)

duty_cycle = 4000
power = 0
on = 0
q=[]
pwm = int(duty_cycle * power)


def vel_callback(msg):
  global px,py,pth,x,y,th,vx,vy,vth,current_time,last_time
  max_pwm = 255
  max_speed = 1.0
  mas_turn = 1.0



  def speed_to_pwm(speed):
	pwm = int((speed / max_speed) * max_pwm)
	#pwm = max(0, min(max_pwm,pwm))
	return pwm
  #wf1_v = (msg.linear.x-msg.linear.y-msg.angular.z * 0.04) / 0.03
  #wf2_v = (msg.linear.x+msg.linear.y+msg.angular.z * 0.04) / 0.03
  #wf3_v = (msg.linear.x+msg.linear.y-msg.angular.z * 0.04) / 0.03
  #wf4_v = (msg.linear.x-msg.linear.y+msg.angular.z * 0.04) / 0.03
  wf1_v = (1/0.04)*((msg.linear.x-msg.linear.y)-(0.25+0.24)*msg.angular.z)
  wf2_v = (1/0.04)*((msg.linear.x+msg.linear.y)+(0.25+0.24)*msg.angular.z)
  wf3_v = (1/0.04)*((msg.linear.x+msg.linear.y)-(0.25+0.24)*msg.angular.z)
  wf4_v = (1/0.04)*((msg.linear.x-msg.linear.y)+(0.25+0.24)*msg.angular.z)
  x = (wf1_v+wf2_v+wf3_v+wf4_v)*0.04/4
  y = (-wf1_v+wf2_v+wf3_v-wf4_v)*0.04/4
  th = (-wf1_v+wf2_v-wf3_v+wf4_v)*0.04/4*(0.12+0.12)
  wf1 = speed_to_pwm(wf1_v)
  wf2 = speed_to_pwm(wf2_v)
  wf3 = speed_to_pwm(wf3_v)
  wf4 = speed_to_pwm(wf4_v)
    
  print(wf1_v,wf2_v,wf3_v,wf4_v)

  def WFL1(LF,LB):
  	PWM.set_pwm(12, on, int(LF))
  	PWM.set_pwm(13, on, int(LB))
  def WFR1(LF,LB):
  	PWM.set_pwm(14, on, int(LF))
  	PWM.set_pwm(15, on, int(LB))
  
  def WFL2(LF,LB):
  	PWM.set_pwm(10, on, int(LF))
  	PWM.set_pwm(11, on, int(LB))
  def WFR2(LF,LB):
  	PWM.set_pwm(8, on, int(LF))
  	PWM.set_pwm(9, on, int(LB))
  
  if wf1>0:
  	WFL1(abs(wf1),0) 
  if wf1<=0:
  	WFL1(0,abs(wf1)) 
  if wf2>0:
  	WFR1(abs(wf2),0) 
  if wf2<=0:
  	WFR1(0,abs(wf2)) 
  if wf3>0:
  	WFL2(abs(wf3),0) 
  if wf3<=0:
  	WFL2(0,abs(wf3)) 
  if wf4>0:
  	WFR2(abs(wf4),0) 
  if wf4<=0:
  	WFR2(0,abs(wf4))

       

if __name__ =='__main__':
	#try:
		sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
		rospy.init_node('publisher_to_my_rovot_test', anonymous=True)
		rate = rospy.Rate(10)
		current_time = rospy.Time.now()
		last_time = rospy.Time.now()
		x = 0.0
		y = 0.0
		th = 0.0
		px = 0.0
		py = 0.0
		pth = 0.0
		t = 0.0 
		vx = 0.0
		vy = 0.0
		vth = 0.0
		#odom_pp()
		rospy.spin()
			
			
	#except rospy.ROSInterruptException:
		#pass
##########


