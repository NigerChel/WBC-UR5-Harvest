#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, TransformStamped
import tf2_ros

class PoseArrayToTF2Broadcaster:
    def __init__(self):
        self.pose_array_sub = rospy.Subscriber('/pose_array_camera', PoseArray, self.pose_array_callback)
        self.br = tf2_ros.TransformBroadcaster()

    def pose_array_callback(self, msg):
        i = 0
        for pose in msg.poses:
            i = i + 1
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "camera_link"  # Change this to the parent frame you are using
            t.child_frame_id = "pose_" + str(i)

            t.transform.translation.x = pose.position.x #* 0.001
            t.transform.translation.y = pose.position.y #* 0.001
            t.transform.translation.z = pose.position.z #* 0.001

            q = pose.orientation
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = 1.0 #q.w

            self.br.sendTransform(t)
            rospy.loginfo("Broadcasting transform for frame pose_{}".format(i))

if __name__ == '__main__':
    rospy.init_node('pose_array_to_tf2_broadcaster')
    PoseArrayToTF2Broadcaster()
    rospy.spin()

