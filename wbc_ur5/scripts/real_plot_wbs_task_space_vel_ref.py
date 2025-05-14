#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import csv

# Initialize lists to store time and velocity data
time_data = []
gripper_velocities = {'x': [], 'y': [], 'z': []}
velocity_ref_data = {'x': [], 'y': [], 'z': []}

# Initialize reference velocities
velocity_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0}

def callback_reference_velocity(msg):
    """Callback function for the /cmd_vel topic."""
    global velocity_ref
    # Update reference velocities from incoming Twist message
    velocity_ref['x'] = msg.linear.x
    velocity_ref['y'] = msg.linear.y
    velocity_ref['z'] = msg.linear.z

def callback_whole_body(msg):
    """Callback function for /whole_body_state_topic."""
    global time_data, gripper_velocities, velocity_ref_data
    current_time = time.time() - start_time  # Calculate elapsed time since the script started

    # Append the elapsed time to the list
    time_data.append(current_time)

    # Extract gripper linear velocity
    for contact in msg.contacts:
        if contact.name == "wrist_3_joint":  # Replace with the appropriate contact link name
            gripper_velocities['x'].append(contact.velocity.linear.x)
            gripper_velocities['y'].append(contact.velocity.linear.y)
            gripper_velocities['z'].append(contact.velocity.linear.z)
            break

    # Append current reference values to their respective lists
    velocity_ref_data['x'].append(velocity_ref['x'])
    velocity_ref_data['y'].append(velocity_ref['y'])
    velocity_ref_data['z'].append(velocity_ref['z'])

def save_data_to_csv():
    """Save the collected gripper velocities and reference velocities to a CSV file."""
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_velocities.csv'

    # Write data to a CSV file
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        # Write the header
        header = ['Time (s)', 'Gripper Velocity X', 'Gripper Velocity Y', 'Gripper Velocity Z',
                  'Reference Velocity X', 'Reference Velocity Y', 'Reference Velocity Z']
        writer.writerow(header)

        # Determine the maximum length between the datasets to avoid index errors
        max_len = max(len(time_data), len(gripper_velocities['x']))

        # Write rows to the CSV
        for i in range(max_len):
            row = [time_data[i] if i < len(time_data) else '']
            # Gripper velocity data
            row += [gripper_velocities['x'][i] if i < len(gripper_velocities['x']) else '',
                    gripper_velocities['y'][i] if i < len(gripper_velocities['y']) else '',
                    gripper_velocities['z'][i] if i < len(gripper_velocities['z']) else '']
            # Reference velocity data
            row += [velocity_ref_data['x'][i], velocity_ref_data['y'][i], velocity_ref_data['z'][i]]
            writer.writerow(row)

    print("Data saved to CSV successfully!")

def plot_data():
    """Plot the gripper velocity and reference velocity data."""
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot actual gripper velocities
    ax.plot(time_data, gripper_velocities['x'], label='Gripper Velocity X', color='red')
    ax.plot(time_data, gripper_velocities['y'], label='Gripper Velocity Y', color='green')
    ax.plot(time_data, gripper_velocities['z'], label='Gripper Velocity Z', color='blue')

    # Plot reference velocities
    ax.plot(time_data, velocity_ref_data['x'], '--', color='red', label='Reference Velocity X')
    ax.plot(time_data, velocity_ref_data['y'], '--', color='green', label='Reference Velocity Y')
    ax.plot(time_data, velocity_ref_data['z'], '--', color='blue', label='Reference Velocity Z')

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Gripper Velocity vs. Reference Velocity over Time')
    ax.legend()
    ax.grid()

    # Save the plot to a file
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_velocity_reference.png', dpi=300)
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/gripper_velocity_reference.svg')
    print("Plot saved successfully!")

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('velocity_plotter', anonymous=True)

    # Record the start time
    start_time = time.time()

    # Subscribe to the reference velocity topic
    rospy.Subscriber('/cmd_vel', Twist, callback_reference_velocity)

    # Subscribe to the ROS topic for whole body state
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)

    # Allow some time for data collection
    collection_duration = rospy.get_param('~collection_duration', 200)
    rospy.sleep(collection_duration)

    plot_data()  # Display and save the plot after data collection
    save_data_to_csv()  # Save data to CSV after plotting

if __name__ == '__main__':
    main()

