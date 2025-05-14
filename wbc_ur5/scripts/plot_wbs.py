#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState  # Import your custom message type
import matplotlib.pyplot as plt
import time

# Initialize lists to store time and data
time_data = []
joint_positions_data = []

def callback(msg):
    """Callback function to process the incoming ROS message."""
    global time_data, joint_positions_data
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Ensure current_time is numeric
    if isinstance(current_time, (int, float)):
        time_data.append(current_time)  # Append the elapsed time to the list

    # Check if 'joints' is not empty and 'position' is a numeric type
    if len(msg.joints) > 0 and hasattr(msg.joints[0], 'position'):
        joint_position = msg.joints[0].position  # Extract the position
        if isinstance(joint_position, (int, float)):
            joint_positions_data.append(joint_position)  # Append the joint position to the list

def save_plot():
    """Save the collected data as a plot."""
    plt.figure()
    plt.plot(time_data, joint_positions_data, label='Joint 1 Position')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Joint Position (radians)')
    plt.title('Joint Position Over Time')
    plt.legend(loc='upper right')
    plt.grid()

    # Set Y-axis limits and intervals
    plt.ylim([0, 2])  # Set the y-axis range from -2 to 2

    # Save the plot with a high resolution
    plt.savefig('joint_position_plot.png', dpi=300)  # Save as PNG with 300 DPI for high quality
    plt.savefig('joint_position_plot.pdf')  # Save as PDF for vector graphics
    plt.savefig('joint_position_plot.svg')  # Save as SVG for vector graphics
    print("Plot saved successfully!")

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('matplotlib_ros_plotter', anonymous=True)
    
    # Record the start time
    start_time = time.time()
    
    # Subscribe to the ROS topic of type WholeBodyState
    rospy.Subscriber('/robot_states', WholeBodyState, callback)
    
    # Allow some time for data collection
    rospy.sleep(20)  # Adjust sleep time as needed for data collection
    
    save_plot()  # Save the plot after data collection

if __name__ == '__main__':
    main()



      
