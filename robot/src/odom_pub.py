#!/usr/bin/env python
import math
from math import sin, cos, pi
from sensor_msgs.msg import Imu
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def vel_callback(msg):
	global vx,vy,vth
	#wf1_v = (msg.linear.x-msg.linear.y-msg.angular.z * 0.04) / 0.03
	#wf2_v = (msg.linear.x+msg.linear.y+msg.angular.z * 0.04) / 0.03
	#wf3_v = (msg.linear.x+msg.linear.y-msg.angular.z * 0.04) / 0.03
	#wf4_v = (msg.linear.x-msg.linear.y+msg.angular.z * 0.04) / 0.03
	wf1_v = (1/0.04)*(msg.linear.x-msg.linear.y-(0.12+0.12)*msg.angular.z)
	wf2_v = (1/0.04)*(msg.linear.x+msg.linear.y+(0.12+0.12)*msg.angular.z)
	wf3_v = (1/0.04)*(msg.linear.x+msg.linear.y-(0.12+0.12)*msg.angular.z)
	wf4_v = (1/0.04)*(msg.linear.x-msg.linear.y+(0.12+0.12)*msg.angular.z)
	vx = (wf1_v+wf2_v+wf3_v+wf4_v)*0.08/4
	vy = (-wf1_v+wf2_v+wf3_v-wf4_v)*0.08/4
	#vth = (-wf1_v+wf2_v-wf3_v+wf4_v)*(0.04/4)*(0.12+0.12)
	
def imu_callback(msg):
  global gyro_z,accel_x,accel_y
  gyro_z = round (msg.angular_velocity.z,1)
  accel_x = msg.linear_acceleration.x
  accel_y = msg.linear_acceleration.y


       

if __name__ =='__main__':
  rospy.init_node('odometry_publisher')
  sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
  sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
  odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
  odom_broadcaster = tf.TransformBroadcaster()
  x = 0.0
  y = 0.0
  th = 0.0
  vx =0.0
  vy = 0.0
  vth = 0.0
  accel_x = 0.0
  accel_y = 0.0
  gyro_z = 0.0
  current_time = rospy.Time.now()
  last_time = rospy.Time.now()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    #vx = accel_x * dt
    #vy = accel_y * dt
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt 
    #delta_th = vth * dt * 20.0
    vth = gyro_z * dt
    delta_th = vth
    
    x += delta_x
    y += delta_y
    th += delta_th
    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
    (x, y, 0.),
    odom_quat,
    current_time,
    "base_footprint",
    "odom"
    )
    
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    
    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    
    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    # publish the message
    odom_pub.publish(odom)
    
    last_time = current_time
    r.sleep()


