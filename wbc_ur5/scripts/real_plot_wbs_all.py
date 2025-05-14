#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
import matplotlib.pyplot as plt
import time
import yaml
import numpy as np
import csv

# Initialize lists to store time and data for each joint
time_data_reemc = []
joint_positions_data_reemc = {i: [] for i in range(6)}  # For /reemc/positions
time_data_whole_body = []
joint_positions_data_whole_body = {i: [] for i in range(6)}  # For /whole_body_state_topic

# Initialize lists to store dynamic q_ref, q_lb, and q_ub values over time
q_ref_data = {i: [] for i in range(6)}
q_lb_data = {i: [] for i in range(6)}
q_ub_data = {i: [] for i in range(6)}

# Define the path to your YAML file
yaml_file_path = "/home/niger/ros_ws/src/wbc_ur5/config/real_param_position_ur5_joint_limit_mod1.yaml"

# Function to load configuration dynamically from YAML
def load_yaml_config():
    global q_ref, q_lb, q_ub
    try:
        with open(yaml_file_path, 'r') as file:
            conf = yaml.safe_load(file)

        # Populate q_ref and joint limits from configuration
        for i in range(6):
            q_ref[i] = conf["q_joint_ref"]["arm_left"]["j{}".format(i + 1)]
            q_lb[i] = conf["q_joint_ref"]["lb_joint_angle"]["j{}".format(i + 1)] * (np.pi / 180.0)
            q_ub[i] = conf["q_joint_ref"]["ub_joint_angle"]["j{}".format(i + 1)] * (np.pi / 180.0)

        print("Configuration reloaded successfully!")
    except Exception as e:
        rospy.logerr("Error loading YAML file: {}".format(e))

# Reload the YAML configuration periodically
def periodic_reload_config(interval=1):
    rospy.Timer(rospy.Duration(interval), lambda event: load_yaml_config())

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

    # Also save the dynamic values of q_ref, q_lb, and q_ub at each time step
    for i in range(6):
        q_ref_data[i].append(q_ref[i])
        q_lb_data[i].append(q_lb[i])
        q_ub_data[i].append(q_ub[i])

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

def save_data_to_csv():
    """Save the collected joint positions, q_ref, q_lb, and q_ub to a CSV file."""
    # Path to save the CSV file
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_joint_limit/joint_positions.csv'

    # Write data to a CSV file
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        # Write the header
        header = ['Time (s)', 
                  'Joint 1 Position REEMC', 'Joint 1 Position Whole Body', 'q_ref Joint 1', 'q_lb Joint 1', 'q_ub Joint 1',
                  'Joint 2 Position REEMC', 'Joint 2 Position Whole Body', 'q_ref Joint 2', 'q_lb Joint 2', 'q_ub Joint 2',
                  'Joint 3 Position REEMC', 'Joint 3 Position Whole Body', 'q_ref Joint 3', 'q_lb Joint 3', 'q_ub Joint 3',
                  'Joint 4 Position REEMC', 'Joint 4 Position Whole Body', 'q_ref Joint 4', 'q_lb Joint 4', 'q_ub Joint 4',
                  'Joint 5 Position REEMC', 'Joint 5 Position Whole Body', 'q_ref Joint 5', 'q_lb Joint 5', 'q_ub Joint 5',
                  'Joint 6 Position REEMC', 'Joint 6 Position Whole Body', 'q_ref Joint 6', 'q_lb Joint 6', 'q_ub Joint 6']
        writer.writerow(header)

        # Determine the maximum length between the datasets to avoid index errors
        max_len = max(len(time_data_reemc), len(time_data_whole_body))

        # Write rows to the CSV
        for i in range(max_len):
            # Initialize row with time
            row = [time_data_reemc[i] if i < len(time_data_reemc) else '']

            # Append joint position data for each joint from both reemc and whole body
            for j in range(6):
                # REEMC data
                reemc_pos = joint_positions_data_reemc[j][i] if i < len(joint_positions_data_reemc[j]) else ''
                # Whole body data
                whole_body_pos = joint_positions_data_whole_body[j][i] if i < len(joint_positions_data_whole_body[j]) else ''
                # q_ref, q_lb, and q_ub for each joint
                row += [reemc_pos, whole_body_pos, q_ref_data[j][i], q_lb_data[j][i], q_ub_data[j][i]]

            # Write the row to the CSV file
            writer.writerow(row)

    print("Data saved to CSV successfully!")

def plot_subplots():
    """Plot the joint positions data in subplots and save the plots as images."""
    fig, axes = plt.subplots(3, 2, figsize=(12, 12))  # Create a 3x2 grid of subplots

    # Plot each joint's position data in its respective subplot
    for i in range(6):
        ax = axes[i // 2, i % 2]  # Determine subplot position
        # Plot data from /reemc/positions
        if time_data_reemc:
            ax.plot(time_data_reemc, joint_positions_data_reemc[i], label='$q_d$ (QP solver)', color='black', linestyle='-')
        # Plot data from /whole_body_state_topic
        if time_data_whole_body:
            ax.plot(time_data_whole_body, joint_positions_data_whole_body[i], label='$q$', color='blue', linestyle='--')
        
        ax.plot(time_data_reemc, q_ref_data[i], color='green', linestyle='-.', label='$q_{ref}$')  # Plot q_ref dynamically
        ax.plot(time_data_reemc, q_lb_data[i], color='r', linestyle='--', label='$q_{lb}$')  # Plot lower bound dynamically
        ax.plot(time_data_reemc, q_ub_data[i], color='r', linestyle='--', label='$q_{ub}$')  # Plot upper bound dynamically
        ax.set_xlabel('Time (seconds)', fontsize=10)
        ax.set_ylabel('Position (radians)', fontsize=10)
        ax.set_title('Joint {} Position'.format(i + 1), fontsize=12)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, linewidth=0.5)

    plt.tight_layout()  # Adjust subplots to fit into figure area.
    # Save the plot to a file
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_joint_limit/joint_positions_plot.png')
    plt.show()

def main():
    global q_ref, q_lb, q_ub, start_time

    rospy.init_node('joint_positions_plotter', anonymous=True)

    # Initialize q_ref, q_lb, and q_ub with initial values (ensure correct length)
    q_ref = [0.0] * 6
    q_lb = [0.0] * 6
    q_ub = [0.0] * 6

    # Start time for tracking
    start_time = time.time()

    # Subscribe to /reemc/positions and /whole_body_state_topic
    rospy.Subscriber('/reemc/positions', WholeBodyState, callback_reemc)
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)

    # Reload YAML configuration periodically
    periodic_reload_config(interval=1)

    # Allow some time for data collection
    rospy.sleep(20)  # Adjust sleep time as needed for data collection

    # After stopping ROS, save the data and generate plots
    save_data_to_csv()
    plot_subplots()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Node terminated.")













      
