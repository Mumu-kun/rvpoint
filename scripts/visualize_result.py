import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def load_pcd(filepath):
    points = []
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    data_started = False
    for line in lines:
        if line.startswith('DATA'):
            data_started = True
            continue
        if not data_started or not line.strip():
            continue
            
        parts = line.split()
        if len(parts) >= 3:
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                points.append((x, y, z))
            except ValueError:
                continue
    return points

def visualize(filepath, output_image):
    points = load_pcd(filepath)
    if not points:
        print(f"No points loaded from {filepath}")
        return

    print(f"Loaded {len(points)} points.")
    
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(xs, ys, zs, c=zs, cmap='viridis', s=2)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Point Cloud Visualization: {os.path.basename(filepath)}')
    
    # Equal aspect ratio hack
    max_range = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)) / 2.0
    mid_x = (max(xs)+min(xs)) * 0.5
    mid_y = (max(ys)+min(ys)) * 0.5
    mid_z = (max(zs)+min(zs)) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.savefig(output_image)
    print(f"Saved visualization to {output_image}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 visualize_result.py <input.pcd> <output.png>")
        sys.exit(1)
        
    input_pcd = sys.argv[1]
    output_png = sys.argv[2]
    visualize(input_pcd, output_png)
