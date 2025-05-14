#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64
import numpy as np
from scipy.interpolate import CubicSpline
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PoseInterpolator:
    def __init__(self):
        # ROS publishers
        self.pose_pub = rospy.Publisher('/interpolated_pose', Pose, queue_size=10)
        self.velocity_pub = rospy.Publisher('/interpolated_velocity', Float64, queue_size=10)
        self.acceleration_pub = rospy.Publisher('/interpolated_acceleration', Float64, queue_size=10)
        
        # Define waypoints and times (position and orientation)
        waypoints = [
            ([0, 0, 0], [0, 0, 0, 1]),    # (position, quaternion)
            ([1, 2, 3], [0.707, 0, 0, 0.707]),
            ([4, 5, 6], [1, 0, 0, 0]),
            ([7, 8, 9], [0, 0.707, 0, 0.707])
        ]
        time_intervals = np.array([0, 1, 2, 3])
        
        # Separate positions and orientations
        positions = np.array([wp[0] for wp in waypoints])
        orientations = np.array([euler_from_quaternion(wp[1]) for wp in waypoints])  # Convert quaternions to Euler angles
        
        # Create splines for position and orientation
        self.spl_x = CubicSpline(time_intervals, positions[:, 0], bc_type='clamped')
        self.spl_y = CubicSpline(time_intervals, positions[:, 1], bc_type='clamped')
        self.spl_z = CubicSpline(time_intervals, positions[:, 2], bc_type='clamped')
        self.spl_roll = CubicSpline(time_intervals, orientations[:, 0], bc_type='clamped')
        self.spl_pitch = CubicSpline(time_intervals, orientations[:, 1], bc_type='clamped')
        self.spl_yaw = CubicSpline(time_intervals, orientations[:, 2], bc_type='clamped')
        
        self.start_time = rospy.Time.now().to_sec()

    def interpolate(self):
        current_time = rospy.Time.now().to_sec() - self.start_time
        if current_time > 20.0:  # assuming the total duration is 3 seconds
            return

        # Calculate interpolated position
        x = self.spl_x(current_time)
        y = self.spl_y(current_time)
        z = self.spl_z(current_time)
        point = Point(x, y, z)
        
        # Calculate interpolated orientation
        roll = self.spl_roll(current_time)
        pitch = self.spl_pitch(current_time)
        yaw = self.spl_yaw(current_time)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        orientation = Quaternion(*quaternion)
        
        # Create pose message
        pose = Pose()
        pose.position = point
        pose.orientation = orientation
        
        # Calculate velocity (norm of position velocity and angular velocity)
        vx = self.spl_x(current_time, 1)
        vy = self.spl_y(current_time, 1)
        vz = self.spl_z(current_time, 1)
        velocity = np.linalg.norm([vx, vy, vz])
        
        v_roll = self.spl_roll(current_time, 1)
        v_pitch = self.spl_pitch(current_time, 1)
        v_yaw = self.spl_yaw(current_time, 1)
        angular_velocity = np.linalg.norm([v_roll, v_pitch, v_yaw])
        
        # Calculate acceleration (norm of position acceleration and angular acceleration)
        ax = self.spl_x(current_time, 2)
        ay = self.spl_y(current_time, 2)
        az = self.spl_z(current_time, 2)
        acceleration = np.linalg.norm([ax, ay, az])
        
        a_roll = self.spl_roll(current_time, 2)
        a_pitch = self.spl_pitch(current_time, 2)
        a_yaw = self.spl_yaw(current_time, 2)
        angular_acceleration = np.linalg.norm([a_roll, a_pitch, a_yaw])
        
        # Publish the results
        self.pose_pub.publish(pose)
        self.velocity_pub.publish(Float64(velocity))
        self.acceleration_pub.publish(Float64(acceleration))

if __name__ == '__main__':
    rospy.init_node('pose_interpolator')
    interpolator = PoseInterpolator()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        interpolator.interpolate()
        rate.sleep()

      
