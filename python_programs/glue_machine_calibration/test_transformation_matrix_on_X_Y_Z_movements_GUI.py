#!/usr/bin/env python3

import numpy as np
import json
import plotly.graph_objects as go

# Load transformation matrix
with open('glue_machine_transformation_matrix.json', 'r') as f:
    data = json.load(f)
    transform_matrix = np.array(data['transformation_matrix'])

# Create cube vertices (200mm size)
cube_size = 200
half_cube_size = cube_size / 2
third_cube_size = cube_size / 3
z_height = cube_size / 5
vertices = np.array([
    [-half_cube_size, -half_cube_size, 0],
    [half_cube_size, -half_cube_size, 0],  # Point along x
    [-half_cube_size, half_cube_size, 0],  # Point along y
    [half_cube_size, half_cube_size, 0],  # Point in xy plane
    [-third_cube_size, -third_cube_size, z_height],  # Point along z
    [third_cube_size, -third_cube_size, z_height],  # Point in xz plane
    [-third_cube_size, third_cube_size, z_height],  # Point in yz plane
    [third_cube_size, third_cube_size, z_height]  # Point in xyz
])

# Function to transform points
def transform_point(point, matrix):
    # Convert to homogeneous coordinates
    homogeneous_point = np.append(point, 1)
    # Apply transformation
    transformed = matrix @ homogeneous_point
    # Convert back to 3D coordinates
    return transformed[:3]

# Transform all vertices
transformed_vertices = np.array([transform_point(vertex, transform_matrix) for vertex in vertices])

# Print coordinates
print("Original Cube Coordinates:")
for i, vertex in enumerate(vertices):
    print(f"Point {i}: {vertex}")

print("\nTransformed Cube Coordinates:")
for i, vertex in enumerate(transformed_vertices):
    print(f"Point {i}: {vertex}")

# Define the edges connecting the vertices
edges = [
    (0, 1), (0, 2), (0, 4),   # Edges from origin
    (1, 3), (1, 5),           # Edges from x-point
    (2, 3), (2, 6),           # Edges from y-point
    (3, 7),                   # Edge from xy-point
    (4, 5), (4, 6),           # Edges from z-point
    (5, 7),                   # Edge from xz-point
    (6, 7)                    # Edge from yz-point
]

# Function to create trace for cube
def create_cube_trace(vertices, color, name):
    lines = []
    for edge in edges:
        x_coords = [vertices[edge[0], 0], vertices[edge[1], 0], None]
        y_coords = [vertices[edge[0], 1], vertices[edge[1], 1], None]
        z_coords = [vertices[edge[0], 2], vertices[edge[1], 2], None]
        
        lines.extend(list(zip(x_coords, y_coords, z_coords)))
    
    # Separate lists for x, y, z
    x_lines, y_lines, z_lines = zip(*lines)
    
    trace = go.Scatter3d(
        x=x_lines,
        y=y_lines,
        z=z_lines,
        mode='lines',
        line=dict(color=color, width=5),
        name=name
    )
    
    return trace

# Create traces for the original and transformed cubes
original_cube_trace = create_cube_trace(vertices, 'blue', 'Original Cube')
transformed_cube_trace = create_cube_trace(transformed_vertices, 'red', 'Transformed Cube')

# Create the layout
layout = go.Layout(
    title='Original (Blue) vs Transformed (Red) Cube',
    scene=dict(
        xaxis=dict(title='X (mm)'),
        yaxis=dict(title='Y (mm)'),
        zaxis=dict(title='Z (mm)'),
        aspectmode='data'  # Ensures equal aspect ratio
    ),
    showlegend=True
)

# Create the figure and add traces
fig = go.Figure(data=[original_cube_trace, transformed_cube_trace], layout=layout)

# Show the interactive plot
fig.show()