#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
import matplotlib.pyplot as plt
import time
import yaml
import numpy as np

# Enable LaTeX rendering
#plt.rc('text', usetex=True)
#plt.rc('font', family='serif')

# Initialize lists to store time and data for each joint
time_data_reemc = []
joint_positions_data_reemc = {i: [] for i in range(6)}  # For /reemc/positions
time_data_whole_body = []
joint_positions_data_whole_body = {i: [] for i in range(6)}  # For /whole_body_state_topic

# Initialize reference positions and joint limits
q_ref = np.zeros(6)
q_lb = np.zeros(6)
q_ub = np.zeros(6)

# Define the path to your YAML file
yaml_file_path = "/home/niger/ros_ws/src/wbc_ur5/config/param_position_ur5_joint_limit_mod1.yaml"

# Load configuration from YAML file
with open(yaml_file_path, 'r') as file:
    conf = yaml.safe_load(file)

# Populate q_ref and joint limits from configuration
for i in range(6):
    q_ref[i] = conf["q_joint_ref"]["arm_left"]["j{}".format(i + 1)]
    q_lb[i] = conf["q_joint_ref"]["lb_joint_angle"]["j{}".format(i + 1)] * (np.pi / 180.0)
    q_ub[i] = conf["q_joint_ref"]["ub_joint_angle"]["j{}".format(i + 1)] * (np.pi / 180.0)

def callback_reemc(msg):
    """Callback function for /reemc/positions."""
    global time_data_reemc, joint_positions_data_reemc
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Ensure current_time is numeric
    if isinstance(current_time, (int, float)):
        time_data_reemc.append(current_time)  # Append the elapsed time to the list

    # Extract joint positions for each joint
    for i in range(6):
        if len(msg.joints) > i and hasattr(msg.joints[i], 'position'):
            joint_position = msg.joints[i].position
            if isinstance(joint_position, (int, float)):
                joint_positions_data_reemc[i].append(joint_position)  # Append the joint position to the list

def callback_whole_body(msg):
    """Callback function for /whole_body_state_topic."""
    global time_data_whole_body, joint_positions_data_whole_body
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Ensure current_time is numeric
    if isinstance(current_time, (int, float)):
        time_data_whole_body.append(current_time)  # Append the elapsed time to the list

    # Extract joint positions for each joint
    for i in range(6):
        if len(msg.joints) > i and hasattr(msg.joints[i], 'position'):
            joint_position = msg.joints[i].position
            if isinstance(joint_position, (int, float)):
                joint_positions_data_whole_body[i].append(joint_position)  # Append the joint position to the list

def plot_subplots():
    """Plot the joint positions data in subplots and save the plots as images."""
    fig, axes = plt.subplots(3, 2, figsize=(12, 12))  # Create a 3x2 grid of subplots

    # Plot each joint's position data in its respective subplot
    for i in range(6):
        ax = axes[i // 2, i % 2]  # Determine subplot position
        # Plot data from /reemc/positions
        if time_data_reemc:
            ax.plot(time_data_reemc, joint_positions_data_reemc[i], label='$q_d$ (QP solver)', color='black', linestyle='-')
            #ax.plot(time_data_reemc, joint_positions_data_reemc[i], label='QP solver {}'.format(i + 1), color='blue')
        # Plot data from /whole_body_state_topic
        if time_data_whole_body:
            ax.plot(time_data_whole_body, joint_positions_data_whole_body[i], label='$q$', color='blue', linestyle='--')
            #ax.plot(time_data_whole_body, joint_positions_data_whole_body[i], label='Current Joint {} '.format(i + 1), color='green', linestyle='--')
        
        ax.axhline(y=q_ref[i], color='green', linestyle='-.', label='$q_{ref}$')  # Plot q_ref
        ax.axhline(y=q_lb[i], color='r', linestyle='--', label='$q_{lb}$')  # Plot lower bound
        ax.axhline(y=q_ub[i], color='r', linestyle='--', label='$q_{ub}$')  # Plot upper bound
        ax.set_xlabel('Time (seconds)', fontsize=10)
        ax.set_ylabel('Position (radians)', fontsize=10)
        ax.set_title('Joint {} Position'.format(i + 1), fontsize=12)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, linewidth=0.5)

    plt.tight_layout()  # Adjust subplots to fit into figure area.
    # Save the plot to a file
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_joint_limit/joint_positions.png', dpi=300)
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_joint_limit/joint_positions.svg')
    print("Plot saved successfully!")
    #plt.show()  # Display the plot

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('matplotlib_ros_plotter', anonymous=True)
    
    # Record the start time
    start_time = time.time()
    
    # Subscribe to both ROS topics of type WholeBodyState
    rospy.Subscriber('/reemc/positions', WholeBodyState, callback_reemc)
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)
    
    # Allow some time for data collection
    rospy.sleep(10)  # Adjust sleep time as needed for data collection
    
    plot_subplots()  # Display and save the plot after data collection

if __name__ == '__main__':
    main()









      
