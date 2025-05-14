#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import time
import csv

# Initialize lists to store time and data for position and orientation
time_data = []
gripper_positions = []
gripper_orientations = []

# Initialize lists to store reference position and orientation over time
position_ref_data = {'x': [], 'y': [], 'z': []}
orientation_ref_data = {'x': [], 'y': [], 'z': [], 'w': []}

# Initialize reference position and orientation
position_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0}
orientation_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}

def callback_reference_pose(msg):
    """Callback function for the /multi_reference_left topic."""
    global position_ref, orientation_ref

    # Update position_ref and orientation_ref with incoming message data
    position_ref['x'] = msg.position.x
    position_ref['y'] = msg.position.y
    position_ref['z'] = msg.position.z

    orientation_ref['x'] = msg.orientation.x
    orientation_ref['y'] = msg.orientation.y
    orientation_ref['z'] = msg.orientation.z
    orientation_ref['w'] = msg.orientation.w

def callback_whole_body(msg):
    """Callback function for /whole_body_state_topic."""
    global time_data, gripper_positions, gripper_orientations, position_ref_data, orientation_ref_data
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Append the elapsed time to the list
    time_data.append(current_time)

    # Extract gripper position and orientation
    for contact in msg.contacts:
        if contact.name == "gripper_link": 
            gripper_positions.append((contact.pose.position.x, contact.pose.position.y, contact.pose.position.z))
            gripper_orientations.append((contact.pose.orientation.x, contact.pose.orientation.y, contact.pose.orientation.z, contact.pose.orientation.w))
            break

    # Append current reference values to their respective lists
    position_ref_data['x'].append(position_ref['x'])
    position_ref_data['y'].append(position_ref['y'])
    position_ref_data['z'].append(position_ref['z'])

    orientation_ref_data['x'].append(orientation_ref['x'])
    orientation_ref_data['y'].append(orientation_ref['y'])
    orientation_ref_data['z'].append(orientation_ref['z'])
    orientation_ref_data['w'].append(orientation_ref['w'])

def save_data_to_csv():
    """Save the collected gripper positions and orientations to a CSV file."""
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_task_space/real_gripper_positions_orientations_pd.csv'

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
            row += [position_ref_data['x'][i], position_ref_data['y'][i], position_ref_data['z'][i],
                    orientation_ref_data['w'][i], orientation_ref_data['x'][i], orientation_ref_data['y'][i], orientation_ref_data['z'][i]]
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
    ax1.plot(time_data, position_ref_data['x'], '--', color='red', label='Desired X Position')
    ax1.plot(time_data, position_ref_data['y'], '--', color='green', label='Desired Y Position')
    ax1.plot(time_data, position_ref_data['z'], '--', color='blue', label='Desired Z Position')
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
    ax2.plot(time_data, orientation_ref_data['x'], '--', color='orange', label='Desired X Orientation')
    ax2.plot(time_data, orientation_ref_data['y'], '--', color='purple', label='Desired Y Orientation')
    ax2.plot(time_data, orientation_ref_data['z'], '--', color='cyan', label='Desired Z Orientation')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Orientation (quaternion)')
    ax2.set_title('Gripper Orientation over Time')
    ax2.legend()
    ax2.grid()

    plt.tight_layout()  # Adjust layout
    # Save the plot to a file
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/real_gripper_position_orientation_pd.png', dpi=300)
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/real_gripper_position_orientation_pd.svg')
    print("Plot saved successfully!")

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('task_space_plotter', anonymous=True)

    # Record the start time
    start_time = time.time()

    # Subscribe to the reference pose topic
    rospy.Subscriber('/multi_reference_left', Pose, callback_reference_pose)

    # Subscribe to the ROS topic for whole body state
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)

    # Allow some time for data collection
    rospy.sleep(40)  # Adjust sleep time as needed for data collection

    plot_data()  # Display and save the plot after data collection
    save_data_to_csv()  # Save data to CSV after plotting

if __name__ == '__main__':
    main()
