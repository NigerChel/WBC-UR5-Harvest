#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import Pose
import math
import numpy as np

global position
position = [0.5, 0.5, 0.5] 
def target_callback(msg5):
    position = np.array([msg5.position.x, msg5.position.y, msg5.position.z]) 
    print(position)
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = position[0]/1000
    t.transform.translation.y = position[1]/1000
    t.transform.translation.z = position[2]/1000
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

if __name__ == '__main__':
    rospy.init_node('camera_tf2_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "wrist_3_link"
    t.child_frame_id = "berry"

    rospy.Subscriber('camera_frame', Pose, target_callback, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        

        br.sendTransform(t)
        rate.sleep()
      
