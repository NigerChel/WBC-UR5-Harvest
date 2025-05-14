#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import csv

# Initialize lists to store time and error data
time_data = []
current_error_data = {'x': [], 'y': [], 'z': []}
delayed_error_data = {'x': [], 'y': [], 'z': []}

start_time = None

def callback_current_error(msg):
    """Callback for the /current_error topic."""
    global time_data, current_error_data, start_time
    
    if start_time is None:
        start_time = rospy.Time.now().to_sec()
    
    current_time = rospy.Time.now().to_sec() - start_time
    time_data.append(current_time)
    
    current_error_data['x'].append(msg.x)
    current_error_data['y'].append(msg.y)
    current_error_data['z'].append(msg.z)

def callback_delayed_error(msg):
    """Callback for the /delayed_error topic."""
    global delayed_error_data
    
    delayed_error_data['x'].append(msg.x)
    delayed_error_data['y'].append(msg.y)
    delayed_error_data['z'].append(msg.z)

def save_data_to_csv():
    """Save the collected error data to a CSV file."""
    csv_file_path = '/home/niger/ros_ws/src/wbc_ur5/plots_task_space/error_data.csv'
    
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['Time (s)', 'Current Error X', 'Current Error Y', 'Current Error Z',
                         'Delayed Error X', 'Delayed Error Y', 'Delayed Error Z'])
        
        for i in range(len(time_data)):
            row = [time_data[i]]
            row += [current_error_data['x'][i], current_error_data['y'][i], current_error_data['z'][i]]
            row += [delayed_error_data['x'][i], delayed_error_data['y'][i], delayed_error_data['z'][i]]
            writer.writerow(row)
    
    print("Data saved to CSV successfully!")

def plot_data():
    """Plot the error data to visualize the delay."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    labels = ['X Error', 'Y Error', 'Z Error']
    colors = ['red', 'green', 'blue']
    
    for i, key in enumerate(['x', 'y', 'z']):
        axes[i].plot(time_data, current_error_data[key], label='Current ' + labels[i], color=colors[i])
        axes[i].plot(time_data, delayed_error_data[key], '--', label='Delayed ' + labels[i], color=colors[i])
        axes[i].set_xlabel('Time (seconds)')
        axes[i].set_ylabel('Error (meters)')
        axes[i].set_title(labels[i] + ' Over Time')
        axes[i].legend()
        axes[i].grid()
    
    plt.tight_layout()
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/error_comparison.png', dpi=300)
    plt.savefig('/home/niger/ros_ws/src/wbc_ur5/plots_task_space/error_comparison.svg')
    print("Plot saved successfully!")

def main():
    rospy.init_node('error_plotter', anonymous=True)
    
    rospy.Subscriber('/current_error', Vector3, callback_current_error)
    rospy.Subscriber('/delayed_error', Vector3, callback_delayed_error)
    
    rospy.sleep(40)  # Adjust duration as needed
    
    plot_data()
    save_data_to_csv()

if __name__ == '__main__':
    main()