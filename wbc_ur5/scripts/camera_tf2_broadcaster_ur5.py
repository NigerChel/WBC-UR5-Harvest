#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg

def broadcast_transform():
    rospy.init_node('wrist_3_link_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # Set translation (x, y, z) and rotation (roll, pitch, yaw) for wrist_3_link
        translation = (0.0, -0.6, 0.0)
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        br.sendTransform(
            translation,  # Translation
            rotation,     # Rotation (as quaternion)
            rospy.Time.now(),  # Timestamp
            "camera",    # Child frame
            "wrist_3_link"        # Parent frame
        )
        
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass

      
