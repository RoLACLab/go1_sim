import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

# Directory to save visualizations
VISUALIZATION_DIR = "lidar_visualizations"
os.makedirs(VISUALIZATION_DIR, exist_ok=True)

def create_lidar_visualization_from_json(json_path: str) -> str:
    """
    Loads LiDAR data from JSON, generates a polar plot visualization, and saves the image.
    """
    # Load JSON data
    with open(json_path, 'r') as f:
        data = json.load(f)

    # Extract LiDAR scan data
    ranges = np.array(data['ranges'])
    angle_min = data['angle_min']
    angle_max = data['angle_max']
    range_min_val = data['range_min']
    range_max_val = data['range_max']

    thetas = np.linspace(angle_min, angle_max, len(ranges))

    # Filter invalid ranges
    valid_mask = (ranges > range_min_val) & (ranges < 4)
    ranges_filtered = np.where(valid_mask, ranges, np.nan)

    # Generate file path with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f"lidar_plot_{timestamp}.png"
    file_path = os.path.join(VISUALIZATION_DIR, file_name)

    # Plot configuration
    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'}, facecolor='white')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.scatter(-thetas, ranges_filtered, s=15, c='blue', alpha=0.75)

    # Connect outermost valid points with a line
    valid_indices = np.where(~np.isnan(ranges_filtered))[0]
    if len(valid_indices) >= 2:
        theta1 = -thetas[valid_indices[0]]
        r1 = ranges_filtered[valid_indices[0]]
        theta2 = -thetas[valid_indices[-1]]
        r2 = ranges_filtered[valid_indices[-1]]
        ax.plot([theta1, theta2], [r1, r2], color='blue', linewidth=3)

    # Draw robot as red triangle pointing North
    thetas_robot = [0, -2*np.pi/3, 2*np.pi/3]
    radii_robot = [0.3, 0.15, 0.15]
    ax.fill(thetas_robot, radii_robot, facecolor='red')

    # Final touches
    rmax = np.nanmax(ranges_filtered) if not np.all(np.isnan(ranges_filtered)) else 5
    ax.set_ylim(0, rmax)
    ax.axis('off')

    # Save and return file path
    plt.savefig(file_path, format='png', bbox_inches='tight', pad_inches=0)
    plt.close(fig)
    print(f"Saved: {file_path}")
    return file_path

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python lidar_visualizer.py path_to_lidar_data.json")
    else:
        create_lidar_visualization_from_json(sys.argv[1])