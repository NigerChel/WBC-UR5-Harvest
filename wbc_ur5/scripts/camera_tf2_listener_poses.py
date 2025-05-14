#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseArray, Pose, TransformStamped

class TransformListener:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_array_length = 0
        self.pose_array_sub = rospy.Subscriber('/pose_array_camera', PoseArray, self.pose_array_callback)
        self.pose_array_pub = rospy.Publisher('/transformed_pose_array', PoseArray, queue_size=10)

    def pose_array_callback(self, msg):
        self.pose_array_length = len(msg.poses)
        rospy.loginfo("Received PoseArray xith length: {}".format(self.pose_array_length))

    def lookup_transform(self, target_frame, source_frame):
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
            rospy.loginfo("Transform from {} to {}: {}".format(source_frame, target_frame, trans))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Transform lookup failed: {}".format(e))
            return None

if __name__ == '__main__':
    rospy.init_node('transform_listener')
    listener = TransformListener()
    rospy.sleep(5.0)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.stamp = rospy.Time.now()
        transformed_pose_array.header.frame_id = "base_link"
        try:
            for i in range(listener.pose_array_length):
                trans = listener.lookup_transform("base_link", "pose_" + str(i+1))
                if trans is not None:
                    pose = Pose()
                    pose.position.x = trans.transform.translation.x
                    pose.position.y = trans.transform.translation.y
                    pose.position.z = trans.transform.translation.z
                    pose.orientation = trans.transform.rotation
                    transformed_pose_array.poses.append(pose)
            listener.pose_array_pub.publish(transformed_pose_array)
            rospy.loginfo("Published transformed PoseArray with length: {}".format(len(transformed_pose_array.poses)))
        except Exception as e:
            rospy.logerr("Error while looking up transforms: {}".format(e))
        rate.sleep()


      
