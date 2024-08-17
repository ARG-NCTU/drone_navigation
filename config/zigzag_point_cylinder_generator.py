import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def generate_zigzag_waypoints_cylinder(radius, height, vertical_step, line_distance, num_segments):
    waypoints = []
    
    # Shift the cylinder so the drone starts at [0, 0, 0] facing the cylinder
    initial_angle = 0  # Starting angle on the cylinder
    x_offset = radius
    y_offset = 0
    
    # Start with the first point [0, 0, 0, 0]
    x = 0  # Set to 0 to place it on the surface after the shift
    y = 0
    z = 0
    heading = 0  # Facing the center of the cylinder
    
    segment_angle = 360 / num_segments  # Angle between each vertical segment
    
    for segment in range(num_segments):
        # Determine the current angle
        angle = segment * segment_angle
        
        # Calculate the x, y position on the cylinder's surface
        x = radius * math.cos(math.radians(angle)) - x_offset
        y = radius * math.sin(math.radians(angle)) - y_offset
        
        # Determine the zigzag direction (up or down)
        if segment % 2 == 0:
            # Moving up
            for i in range(0, int(height / vertical_step) + 1):
                z = i * vertical_step
                # Heading towards the cylinder's center (opposite of the normal vector)
                heading = 0 + segment * segment_angle
                if x==0 and y==0 and z==0 and heading==0:
                    pass
                else:
                    waypoints.append([-x, -y, z, heading])
        else:
            # Moving down
            for i in range(int(height / vertical_step), -1, -1):
                z = i * vertical_step
                # Heading towards the cylinder's center (opposite of the normal vector)
                heading = 0 + segment * segment_angle
                if x==0 and y==0 and z==0 and heading==0:
                    pass
                else:
                    waypoints.append([-x, -y, z, heading])
    
    return waypoints

def plot_waypoints_with_headings(waypoints, radius, height):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x_vals = [wp[0] for wp in waypoints]
    y_vals = [wp[1] for wp in waypoints]
    z_vals = [wp[2] for wp in waypoints]
    
    # Plot the zigzag pattern
    ax.plot(x_vals, y_vals, z_vals, marker='o', linestyle='-', color='b')
    
    # Plot the cylinder for reference
    theta = np.linspace(0, 2 * np.pi, 100)
    z_cylinder = np.linspace(0, height, 100)
    theta, z_cylinder = np.meshgrid(theta, z_cylinder)
    x_cylinder = radius * np.cos(theta) - radius
    y_cylinder = radius * np.sin(theta)
    
    ax.plot_surface(-x_cylinder, -y_cylinder, z_cylinder, color='r', alpha=0.3)
    
    # Add arrows to show heading at each waypoint
    arrow_length = 0.5  # Length of the arrow representing the heading
    for wp in waypoints:
        x, y, z, heading = wp
        dx = arrow_length * math.cos(math.radians(heading))
        dy = arrow_length * math.sin(math.radians(heading))
        ax.quiver(x, y, z, dx, dy, 0, color='g', arrow_length_ratio=0.3)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()

def save_waypoints_to_file(waypoints, filename):
    with open(filename, 'w') as file:
        file.write("waypoint: [\n")
        for wp in waypoints:
            line = f"  [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}, {wp[3]:.2f}],\n"
            file.write(line)
        file.write("]\n")

# Example parameters for the zigzag pattern
radius = 5
height = 5
vertical_step = 1  # Distance between each vertical movement
line_distance = 0.5  # Horizontal distance between each vertical line
num_segments = int(2 * math.pi * radius / line_distance)  # Number of vertical lines around the cylinder

# Generate waypoints
waypoints = generate_zigzag_waypoints_cylinder(radius, height, vertical_step, line_distance, num_segments)

# Plot waypoints with headings
plot_waypoints_with_headings(waypoints, radius, height)

# Save waypoints to a file
filename = "waypoint_vertical_pattern_block.yaml"
save_waypoints_to_file(waypoints, filename)

print(f"Waypoints saved to {filename}")