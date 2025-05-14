#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray

def publish_world_effector():
    # Initialize the ROS node
    rospy.init_node('world_effector_publisher')

    # Create a publisher for the /world_effector topic
    pub = rospy.Publisher('/world_effector', PoseArray, queue_size=10)

    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create an instance of the PoseArray message
        pose_array = PoseArray()

        # Create example Pose messages and add them to the array
        pose1 = Pose()
        pose1.position.x = 1.0
        pose1.position.y = 2.0
        pose1.position.z = 3.0
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = 4.0
        pose2.position.y = 5.0
        pose2.position.z = 6.0
        pose2.orientation.x = 0.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 1.0

        # Append the poses to the array
        pose_array.poses.append(pose1)
        pose_array.poses.append(pose2)

        # Publish the pose array
        pub.publish(pose_array)

        # Sleep for the remaining time until we hit our 10 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_world_effector()
    except rospy.ROSInterruptException:
        pass




      
