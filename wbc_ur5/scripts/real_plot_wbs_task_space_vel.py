#!/usr/bin/env python

import rospy
from whole_body_state_msgs.msg import WholeBodyState
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import csv

# Initialize lists to store time and data for linear and angular velocities
time_data = []
linear_velocity_data = {'x': [], 'y': [], 'z': []}
angular_velocity_data = {'x': [], 'y': [], 'z': []}
linear_vel_ref_data = {'x': [], 'y': [], 'z': []}
angular_vel_ref_data = {'x': [], 'y': [], 'z': []}

# Callback functions remain unchanged
def callback_velocity_reference(msg):
    linear_vel_ref_data['x'].append(msg.linear.x)
    linear_vel_ref_data['y'].append(msg.linear.y)
    linear_vel_ref_data['z'].append(msg.linear.z)
    angular_vel_ref_data['x'].append(msg.angular.x)
    angular_vel_ref_data['y'].append(msg.angular.y)
    angular_vel_ref_data['z'].append(msg.angular.z)

def callback_whole_body(msg):
    current_time = time.time() - start_time
    time_data.append(current_time)
    for contact in msg.contacts:
        if contact.name == "wrist_3_joint":
            linear_velocity_data['x'].append(contact.velocity.linear.x)
            linear_velocity_data['y'].append(contact.velocity.linear.y)
            linear_velocity_data['z'].append(contact.velocity.linear.z)
            angular_velocity_data['x'].append(contact.velocity.angular.x)
            angular_velocity_data['y'].append(contact.velocity.angular.y)
            angular_velocity_data['z'].append(contact.velocity.angular.z)
            break

def plot_velocity_data():
    """Plot linear and angular velocity data after simulation duration."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Determine the minimum length to slice the data lists
    min_length = min(len(time_data), len(linear_velocity_data['x']), len(linear_vel_ref_data['x']))
    trimmed_time_data = time_data[:min_length]
    trimmed_linear_velocity_data = {k: v[:min_length] for k, v in linear_velocity_data.items()}
    trimmed_angular_velocity_data = {k: v[:min_length] for k, v in angular_velocity_data.items()}
    trimmed_linear_vel_ref_data = {k: v[:min_length] for k, v in linear_vel_ref_data.items()}
    trimmed_angular_vel_ref_data = {k: v[:min_length] for k, v in angular_vel_ref_data.items()}

    # Plot data after collection
    ax1.plot(trimmed_time_data, trimmed_linear_velocity_data['x'], label='Linear X', color='red')
    ax1.plot(trimmed_time_data, trimmed_linear_velocity_data['y'], label='Linear Y', color='green')
    ax1.plot(trimmed_time_data, trimmed_linear_velocity_data['z'], label='Linear Z', color='blue')
    ax1.plot(trimmed_time_data, trimmed_linear_vel_ref_data['x'], '--', color='red', label='Desired Linear X')
    ax1.plot(trimmed_time_data, trimmed_linear_vel_ref_data['y'], '--', color='green', label='Desired Linear Y')
    ax1.plot(trimmed_time_data, trimmed_linear_vel_ref_data['z'], '--', color='blue', label='Desired Linear Z')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Linear Velocity (m/s)')
    ax1.set_title('Linear Velocity over Time')
    ax1.legend()
    ax1.grid()

    ax2.plot(trimmed_time_data, trimmed_angular_velocity_data['x'], label='Angular X', color='orange')
    ax2.plot(trimmed_time_data, trimmed_angular_velocity_data['y'], label='Angular Y', color='purple')
    ax2.plot(trimmed_time_data, trimmed_angular_velocity_data['z'], label='Angular Z', color='cyan')
    ax2.plot(trimmed_time_data, trimmed_angular_vel_ref_data['x'], '--', color='orange', label='Desired Angular X')
    ax2.plot(trimmed_time_data, trimmed_angular_vel_ref_data['y'], '--', color='purple', label='Desired Angular Y')
    ax2.plot(trimmed_time_data, trimmed_angular_vel_ref_data['z'], '--', color='cyan', label='Desired Angular Z')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Angular Velocity over Time')
    ax2.legend()
    ax2.grid()

    plt.tight_layout()
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/velocity_plot.png', dpi=300)
    plt.show()

def save_velocity_data_to_csv():
    """Save collected velocity data to CSV."""
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_task_space/velocity_data.csv'
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        header = ['Time (s)', 'Linear X', 'Linear Y', 'Linear Z', 'Angular X', 'Angular Y', 'Angular Z', 
                  'Desired Linear X', 'Desired Linear Y', 'Desired Linear Z', 'Desired Angular X', 'Desired Angular Y', 'Desired Angular Z']
        writer.writerow(header)

        max_len = min(len(time_data), len(linear_velocity_data['x']), len(linear_vel_ref_data['x']))
        for i in range(max_len):
            row = [time_data[i]] + [linear_velocity_data['x'][i], linear_velocity_data['y'][i], linear_velocity_data['z'][i]] + \
                  [angular_velocity_data['x'][i], angular_velocity_data['y'][i], angular_velocity_data['z'][i]] + \
                  [linear_vel_ref_data['x'][i], linear_vel_ref_data['y'][i], linear_vel_ref_data['z'][i]] + \
                  [angular_vel_ref_data['x'][i], angular_vel_ref_data['y'][i], angular_vel_ref_data['z'][i]]
            writer.writerow(row)

    print("Velocity data saved to CSV successfully!")

def main():
    global start_time
    rospy.init_node('velocity_plotter', anonymous=True)
    start_time = time.time()

    # Get the simulation duration from parameter or default to 20 seconds
    simulation_duration = 20 #rospy.get_param("~simulation_duration", 20)
    
    rospy.Subscriber('/robot_states', WholeBodyState, callback_whole_body)
    rospy.Subscriber('/cmd_vel', Twist, callback_velocity_reference)

    # Allow time for data collection based on simulation duration
    rospy.sleep(simulation_duration)

    # Plot and save data after collection
    plot_velocity_data()
    save_velocity_data_to_csv()

if __name__ == '__main__':
    main()


