#! /usr/bin/env python3

import random
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Constants for obstacle detection and goal satisfaction
MIN_DISTANCE = 0.5  # Minimum distance to consider an obstacle
GOAL_TOLERANCE = 0.2  # Tolerance for considering the goal reached

def callback_laser_scan(msg):
    ranges = msg.ranges

    # Check if there are any obstacles in front of the robot
    front_obstacle = False
    for i in range(len(ranges) // 3, 2 * len(ranges) // 3):
        if ranges[i] < MIN_DISTANCE:
            front_obstacle = True
            break

    twist = Twist()

    if front_obstacle:
        # If an obstacle is detected, turn to avoid it
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # Adjust the turn angle as needed
    else:
        # If no obstacle, continue moving forward
        twist.linear.x = 0.2
        twist.angular.z = 0.0

    cmd_vel_publisher.publish(twist)

# Define the fitness function
def fitness_function(path):
    total_distance = 0.0
    obstacle_penalty = 0.0

    # Iterate through the path points
    x1, y1 = path[0]
    x2, y2 = path[1]
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    total_distance += distance

    # Check for obstacles using laser scan data
    if obstacle_detected((x1, y1)):
        obstacle_penalty += distance  # Penalize for encountering obstacles

    # Calculate the fitness based on distance and obstacle avoidance
    fitness = total_distance + obstacle_penalty

    # Check if the end position is reached
    end_x, end_y = path[-1]
    goal_distance = ((end_x - x_end) ** 2 + (end_y - y_end) ** 2) ** 0.5
    if goal_distance < GOAL_TOLERANCE:
        fitness *= 0.5  # Give a bonus for reaching the goal

    return fitness

def pso_path_planning():
    # PSO parameters
    num_particles = 30
    max_iterations = 100
    w = 0.5  # Inertia weight
    c1 = 1.5  # Cognitive component
    c2 = 1.5  # Social component

    # Initialize particles with random positions and velocities
    particles = [{'position': (random.uniform(x_start, x_end), random.uniform(y_start, y_end)),
                  'velocity': (random.uniform(-1, 1), random.uniform(-1, 1))}
                 for _ in range(num_particles)]

    # Initialize global best solution
    global_best = None

    # PSO optimization loop
    for iteration in range(max_iterations):
        for particle in particles:
            # Update particle velocity and position (PSO update equations)

            # Evaluate the fitness of the new position
            fitness = fitness_function(particle['position'])

            # Update particle's best position if it's better
            if 'best_fitness' not in particle or fitness > particle['best_fitness']:
                particle['best_position'] = particle['position']
                particle['best_fitness'] = fitness

            # Update global best solution if it's better
            if global_best is None or fitness > global_best['best_fitness']:
                global_best = {'best_position': particle['position'], 'best_fitness': fitness}

    return global_best['best_position']

def main():
    rclpy.init()
    node = rclpy.create_node("pso_path")

    # Create publishers and subscribers
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    laser_scan_subscription = node.create_subscription(LaserScan, '/scan', callback_laser_scan, 10)

    global x_start, y_start, x_end, y_end
    x_start, y_start = 0, 0  # Define the start position
    x_end, y_end = 7, 7  # Define the end position

    # Calculate a path using PSO
    optimized_path = pso_path_planning()

    # Control the robot to follow the optimized path
    for position in optimized_path:
        twist = Twist()
        twist.linear.x = 0.2  # Adjust the linear and angular velocities as needed
        twist.angular.z = 0.0
        cmd_vel_publisher.publish(twist)
        rclpy.spin_once(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
