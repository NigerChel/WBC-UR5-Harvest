#!/usr/bin/env python

import rospy
from visp_hand2eye_calibration.srv import *
from geometry_msgs.msg import Transform

def hand2eye_calibration_client():
    rospy.wait_for_service('hand_eye_calibration')
    
    try:
        hand_eye_calibration = rospy.ServiceProxy('hand_eye_calibration', compute_effector_camera_quick)
        
        # Example input data for the service
        camera_to_object = Transform()
        camera_to_object.translation.x = 0.1
        camera_to_object.translation.y = 0.2
        camera_to_object.translation.z = 0.3
        camera_to_object.rotation.x = 0.0
        camera_to_object.rotation.y = 0.0
        camera_to_object.rotation.z = 0.0
        camera_to_object.rotation.w = 1.0
        
        effector_to_base = Transform()
        effector_to_base.translation.x = 0.4
        effector_to_base.translation.y = 0.5
        effector_to_base.translation.z = 0.6
        effector_to_base.rotation.x = 0.0
        effector_to_base.rotation.y = 0.0
        effector_to_base.rotation.z = 0.0
        effector_to_base.rotation.w = 1.0
        
        # Call the service with the provided data
        response = hand_eye_calibration(camera_to_object, effector_to_base)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('hand2eye_calibration_client')
    
    result = hand2eye_calibration_client()
    
    if result:
        rospy.loginfo("Hand-Eye Calibration Result: \n%s", result)



      
