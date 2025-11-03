#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

class IMUToOdom:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_to_odom')

        # Subscribers
        self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)

        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Time
        self.last_time = rospy.Time.now()

    def imu_callback(self, imu_data):
        current_time = rospy.Time.now()

        # Time difference
        dt = (current_time - self.last_time).to_sec()

        # Linear acceleration in the IMU frame
        accel_x = int (imu_data.linear_acceleration.x)
        accel_y = int (imu_data.linear_acceleration.y)

        # Angular velocity (yaw rate)
        gyro_z = round (imu_data.angular_velocity.z,1)

        # Integrate linear acceleration to compute velocity
        self.vx = accel_x * dt
        self.vy = accel_y * dt

        # Integrate angular velocity to compute orientation (yaw)
        self.vth = gyro_z * dt

        # Compute change in position using the velocities
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_th = self.vth

        # Update position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Create quaternion from the yaw angle
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # Create and publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # Publish odometry
        self.odom_pub.publish(odom)

        # Update time
        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        imu_to_odom = IMUToOdom()
        imu_to_odom.run()
    except rospy.ROSInterruptException:
        pass
