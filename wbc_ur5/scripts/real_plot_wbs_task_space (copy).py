#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
import matplotlib.pyplot as plt
import time
import yaml
import numpy as np
import csv

# Initialize lists to store time and data for position and orientation
time_data = []
gripper_positions = []
gripper_orientations = []

# Initialize reference position and orientation
position_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0}
orientation_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}

# Define the path to your YAML file
yaml_file_path = "/home/niger/ros_ws/src/wbc_ur5/config/real_param_position_ur5_joint_limit_mod1.yaml"

# Function to load configuration dynamically from YAML
def load_yaml_config():
    global position_ref, orientation_ref
    try:
        with open(yaml_file_path, 'r') as file:
            conf = yaml.safe_load(file)

        # Populate position_ref and orientation_ref from configuration
        position_ref['x'] = conf["leg_left"]["position_ref"]["x"]
        position_ref['y'] = conf["leg_left"]["position_ref"]["y"]
        position_ref['z'] = conf["leg_left"]["position_ref"]["z"]

        orientation_ref['w'] = conf["leg_left"]["orientation_ref"]["w"]
        orientation_ref['x'] = conf["leg_left"]["orientation_ref"]["x"]
        orientation_ref['y'] = conf["leg_left"]["orientation_ref"]["y"]
        orientation_ref['z'] = conf["leg_left"]["orientation_ref"]["z"]

        print("Position and orientation references reloaded successfully!")
    except Exception as e:
        rospy.logerr("Error loading YAML file: {}".format(e))

# Reload the YAML configuration periodically
def periodic_reload_config(interval=1):
    rospy.Timer(rospy.Duration(interval), lambda event: load_yaml_config())

def callback_whole_body(msg):
    """Callback function for /whole_body_state_topic."""
    global time_data, gripper_positions, gripper_orientations
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Append the elapsed time to the list
    time_data.append(current_time)

    # Extract gripper position and orientation
    for contact in msg.contacts:
        if contact.name == "gripper_link":
            gripper_positions.append((contact.pose.position.x, contact.pose.position.y, contact.pose.position.z))
            gripper_orientations.append((contact.pose.orientation.x, contact.pose.orientation.y, contact.pose.orientation.z, contact.pose.orientation.w))
            break

def save_data_to_csv():
    """Save the collected gripper positions and orientations to a CSV file."""
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_positions_orientations.csv'

    # Write data to a CSV file
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        # Write the header
        header = ['Time (s)', 
                  'Gripper X', 'Gripper Y', 'Gripper Z',
                  'Gripper Orientation X', 'Gripper Orientation Y', 'Gripper Orientation Z', 'Gripper Orientation W',
                  'Desired Position X', 'Desired Position Y', 'Desired Position Z',
                  'Desired Orientation W', 'Desired Orientation X', 'Desired Orientation Y', 'Desired Orientation Z']
        writer.writerow(header)

        # Determine the maximum length between the datasets to avoid index errors
        max_len = max(len(time_data), len(gripper_positions))

        # Write rows to the CSV
        for i in range(max_len):
            row = [time_data[i] if i < len(time_data) else '']
            # Gripper position data
            if i < len(gripper_positions):
                row += list(gripper_positions[i])
            else:
                row += ['', '', '']
            # Gripper orientation data
            if i < len(gripper_orientations):
                row += list(gripper_orientations[i])
            else:
                row += ['', '', '', '']
            # Desired position and orientation
            row += [position_ref['x'], position_ref['y'], position_ref['z'],
                    orientation_ref['w'], orientation_ref['x'], orientation_ref['y'], orientation_ref['z']]
            writer.writerow(row)

    print("Data saved to CSV successfully!")

def plot_data():
    """Plot the gripper position and orientation data."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 10))  # Create a 2x1 grid of subplots

    # Plot position data
    ax1 = axes[0]
    ax1.plot(time_data, [pos[0] for pos in gripper_positions], label='Gripper X', color='red')
    ax1.plot(time_data, [pos[1] for pos in gripper_positions], label='Gripper Y', color='green')
    ax1.plot(time_data, [pos[2] for pos in gripper_positions], label='Gripper Z', color='blue')
    ax1.axhline(y=position_ref['x'], color='red', linestyle='--', label='Desired X Position')
    ax1.axhline(y=position_ref['y'], color='green', linestyle='--', label='Desired Y Position')
    ax1.axhline(y=position_ref['z'], color='blue', linestyle='--', label='Desired Z Position')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Position (meters)')
    ax1.set_title('Gripper Position over Time')
    ax1.legend()
    ax1.grid()

    # Plot orientation data (using quaternion)
    ax2 = axes[1]
    ax2.plot(time_data, [orient[0] for orient in gripper_orientations], label='Orientation X', color='orange')
    ax2.plot(time_data, [orient[1] for orient in gripper_orientations], label='Orientation Y', color='purple')
    ax2.plot(time_data, [orient[2] for orient in gripper_orientations], label='Orientation Z', color='cyan')
    ax2.axhline(y=orientation_ref['x'], color='orange', linestyle='--', label='Desired X Orientation')
    ax2.axhline(y=orientation_ref['y'], color='purple', linestyle='--', label='Desired Y Orientation')
    ax2.axhline(y=orientation_ref['z'], color='cyan', linestyle='--', label='Desired Z Orientation')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Orientation (quaternion)')
    ax2.set_title('Gripper Orientation over Time')
    ax2.legend()
    ax2.grid()

    plt.tight_layout()  # Adjust layout
    # Save the plot to a file
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_position_orientation.png', dpi=300)
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_position_orientation.svg')
    print("Plot saved successfully!")

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('task_space_plotter', anonymous=True)

    # Record the start time
    start_time = time.time()

    # Load initial configuration from YAML
    load_yaml_config()

    # Set up periodic reloading of the YAML configuration
    periodic_reload_config(interval=1)  # Reload every 10 seconds

    # Subscribe to the ROS topic for whole body state
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)

    # Allow some time for data collection
    rospy.sleep(30)  # Adjust sleep time as needed for data collection

    plot_data()  # Display and save the plot after data collection
    save_data_to_csv()  # Save data to CSV after plotting

if __name__ == '__main__':
    main()









      
