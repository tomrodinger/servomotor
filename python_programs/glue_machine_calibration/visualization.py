# visualization.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize(points_list, labels, filename):
    """Visualize multiple sets of 3D points."""
    colors = ['yellow', 'green', 'blue', 'red', 'cyan', 'magenta', 'black']
    markers = ['o', '^', 's', 'p', '*', 'x', 'D']
    linestyles = ['-', '--', '-.', ':', '-', '--', '-.']

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Define edges for connecting points (assuming square shape)
    edges = [(0,2), (2,1), (1,3), (3,0)]

    for idx, (points, label) in enumerate(zip(points_list, labels)):
        color = colors[idx % len(colors)]
        marker = markers[idx % len(markers)]
        linestyle = linestyles[idx % len(linestyles)]
        # Plot the points
        ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                   color=color, s=100, label=label, marker=marker)

        # Connect the points to form edges if there are at least 4 points
        if points.shape[0] >= 4:
            for start_idx, end_idx in edges:
                start = points[start_idx]
                end = points[end_idx]
                ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                        color=color, linestyle=linestyle, alpha=0.5)

        # Add point labels
        for i, (x, y, z) in enumerate(points):
            ax.text(x, y, z, f'{label[0]}{i+1}', fontsize=8)

    # Customize plot
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('3D Points Visualization')
    ax.legend()
    ax.grid(True)

    # Save plot
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()

