#!/usr/bin/env python

import rosbag
import numpy as np
from nav_msgs.msg import Odometry
import tf
import matplotlib.pyplot as plt

def extract_odometry_data(bag_file, topic_names):
    """
    Extracts x, y, and yaw data from specified odometry topics in a bag file.

    Args:
        bag_file (str): Path to the ROS bag file.
        topic_names (list): List of odometry topic names to extract.

    Returns:
        dict: A dictionary with topic names as keys and numpy arrays of shape (n, 3) as values.
    """
    # Dictionary to store data for each topic
    odometry_data = {topic: [] for topic in topic_names}

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topic_names):
            if topic in topic_names:
                # Extract position (x, y)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y

                # Extract orientation (yaw)
                orientation_q = msg.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                _, _, yaw = tf.transformations.euler_from_quaternion(orientation_list)

                # Append the data
                odometry_data[topic].append([x, y, yaw])

    # Convert lists to numpy arrays
    for topic in topic_names:
        odometry_data[topic] = np.array(odometry_data[topic])

    return odometry_data

def compute_transformation(source, target):
    """
    Computes the rigid transformation (rotation + translation) from source to target.

    Args:
        source (array): Source coordinate as [x, y, yaw].
        target (array): Target coordinate as [x, y, yaw].

    Returns:
        tuple: Translation (dx, dy) and rotation (dtheta).
    """
    dx = target[0] - source[0]
    dy = target[1] - source[1]
    dtheta = target[2] - source[2]
    return dx, dy, dtheta

def apply_transformation(data, dx, dy, dtheta):
    """
    Applies a rigid transformation to the given odometry data.

    Args:
        data (numpy array): Array of shape (n, 3) with [x, y, yaw].
        dx (float): Translation in x.
        dy (float): Translation in y.
        dtheta (float): Rotation angle in radians.

    Returns:
        numpy array: Transformed data of shape (n, 3).
    """
    transformed_data = []
    for x, y, yaw in data:
        # Rotate
        x_new = x * np.cos(dtheta) - y * np.sin(dtheta)
        y_new = x * np.sin(dtheta) + y * np.cos(dtheta)
        # Translate
        x_new += dx
        y_new += dy
        yaw_new = yaw + dtheta
        transformed_data.append([x_new, y_new, yaw_new])
    return np.array(transformed_data)

def plot_odometry_data(odometry_data):
    """
    Plots odometry data on an X-Y plane with yaw represented as arrows.

    Args:
        odometry_data (dict): A dictionary with topic names as keys and numpy arrays of shape (n, 3) as values.
    """
    plt.figure(figsize=(10, 8))

    for topic, data in odometry_data.items():
        x = data[:, 0]
        y = data[:, 1]
        yaw = data[:, 2]

        # Plot positions
        plt.scatter(x, y, label="{}".format(topic), s=10)

        # Plot yaw as arrows
        for i in range(0, len(x), max(1, len(x)//100)):  # Limit number of arrows for clarity
            dx = 0.1 * np.cos(yaw[i])
            dy = 0.1 * np.sin(yaw[i])
            plt.arrow(x[i], y[i], dx, dy, head_width=0.05, head_length=0.1, fc='k', ec='k')

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Odometry Data Visualization")
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    # Path to your bag file
    # bag_file_path = "/home/aristos/catkin_ws/bags/fast_lio_odometry_comparison_full_2025-01-08-12-02-19.bag"
    bag_file_path = "/home/aristos/catkin_ws/bags/fast_lio_odometry_comparison_under_2025-01-08-14-20-47.bag"

    # Topics to extract
    # odometry_topics = ["/aristos/ground_truth", "/Odometry"]
    odometry_topics = ["/aristos/odometry/navsat", "/aristos/ground_truth", "/aristos/odometry/onlyRTK"]

    # Extract data
    odometry_arrays = extract_odometry_data(bag_file_path, odometry_topics)

    # # Transform topic B to the frame of topic A
    # if len(odometry_topics) == 2:
    #     source_topic = odometry_topics[0]
    #     target_topic = odometry_topics[1]

    #     source_start = odometry_arrays[source_topic][0]
    #     target_start = odometry_arrays[target_topic][0]
    #     print(source_start)
    #     print(target_start)

    #     dx, dy, dtheta = compute_transformation(source_start, target_start)
    #     print(dx, dy, dtheta)
    #     odometry_arrays[target_topic] = apply_transformation(odometry_arrays[target_topic], -dx, -dy, -dtheta)
    #     print(odometry_arrays[target_topic])

    # Plot data
    plot_odometry_data(odometry_arrays)