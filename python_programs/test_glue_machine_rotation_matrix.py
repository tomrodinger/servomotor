import numpy as np
import matplotlib.pyplot as plt
import json
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def load_data():
    """Load calibration and transformation data."""
    with open('glue_machine_calibration.json', 'r') as f:
        calibration = json.load(f)
    
    with open('glue_machine_rotation_matrix.json', 'r') as f:
        transform = json.load(f)
    
    return calibration, transform

def points_to_array(data, point_names):
    """Convert points from dictionary to numpy array."""
    points = []
    for name in point_names:
        point = data[name]
        points.append([point['x'], point['y'], point['z']])
    return np.array(points)

def create_theoretical_square(size):
    """Create theoretical square corners centered at origin."""
    half_size = size / 2
    return np.array([
        [-half_size, half_size, 0],    # Top left
        [half_size, half_size, 0],     # Top right
        [half_size, -half_size, 0],    # Bottom right
        [-half_size, -half_size, 0]    # Bottom left
    ])

def apply_transform(points, transform_matrix):
    """Apply 4x4 transformation matrix to points."""
    # Convert to homogeneous coordinates
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    # Apply transform
    transformed = np.dot(homogeneous, transform_matrix.T)
    # Convert back to 3D coordinates
    return transformed[:, :3]

def plot_square(ax, points, color, alpha, label, draw_normals=False):
    """Plot a square from its corner points."""
    # Create the square face
    vertices = [points]
    poly = Poly3DCollection(vertices, alpha=alpha)
    poly.set_facecolor(color)
    ax.add_collection3d(poly)
    
    # Plot the edges
    points_loop = np.vstack([points, points[0]])  # Close the loop
    ax.plot(points_loop[:, 0], points_loop[:, 1], points_loop[:, 2], 
            color=color, label=label)
    
    # Draw normal vectors at corners if requested
    if draw_normals:
        normal = np.cross(points[1] - points[0], points[3] - points[0])
        normal = normal / np.linalg.norm(normal)
        normal *= np.mean([np.linalg.norm(points[i] - points[i-1]) for i in range(1, 4)])
        
        for point in points:
            ax.quiver(point[0], point[1], point[2],
                     normal[0], normal[1], normal[2],
                     color=color, alpha=0.5)

def visualize_transformation():
    # Load data
    calibration, transform_data = load_data()
    transform_matrix = np.array(transform_data['transform_matrix'])
    theoretical_size = transform_data['metadata']['theoretical_square_size']
    
    # Get points
    corner_points = points_to_array(calibration, ['corner1', 'corner2', 'corner3', 'corner4'])
    zref_points = points_to_array(calibration, ['zref1', 'zref2', 'zref3', 'zref4'])
    aligned_corners = np.array(transform_data['metadata']['aligned_corners'])
    
    # Create theoretical square
    theoretical_square = create_theoretical_square(theoretical_size)
    
    # Create figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot original corner points and zref points
    plot_square(ax, corner_points, 'blue', 0.2, 'Original Object')
    ax.scatter(zref_points[:, 0], zref_points[:, 1], zref_points[:, 2], 
              color='red', label='Z Reference Points')
    
    # Plot aligned corners
    plot_square(ax, aligned_corners, 'cyan', 0.3, 'Aligned Object', draw_normals=True)
    
    # Plot theoretical square
    plot_square(ax, theoretical_square, 'green', 0.3, 'Theoretical Square')
    
    # Plot transformed theoretical square
    transformed_square = apply_transform(theoretical_square, np.linalg.inv(transform_matrix))
    plot_square(ax, transformed_square, 'orange', 0.3, 'Transformed Square')
    
    # Customize plot
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Glue Machine Coordinate Transformation Visualization')
    ax.legend()
    
    # Auto scale axes
    all_points = np.vstack([corner_points, zref_points, theoretical_square, transformed_square])
    max_range = np.array([
        all_points[:, 0].max() - all_points[:, 0].min(),
        all_points[:, 1].max() - all_points[:, 1].min(),
        all_points[:, 2].max() - all_points[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Save plot
    plt.savefig('transformation_visualization.png')
    plt.close()

def test_transformation():
    """Test the transformation by applying it to points and checking distances."""
    calibration, transform_data = load_data()
    transform_matrix = np.array(transform_data['transform_matrix'])
    theoretical_size = transform_data['metadata']['theoretical_square_size']
    aligned_corners = np.array(transform_data['metadata']['aligned_corners'])
    
    # Apply transformation to aligned corners
    homogeneous_corners = np.hstack([aligned_corners, np.ones((4, 1))])
    transformed_corners = np.dot(homogeneous_corners, transform_matrix.T)[:, :3]
    
    # Calculate edge lengths and diagonals
    edges = [
        np.linalg.norm(transformed_corners[i] - transformed_corners[(i+1)%4])
        for i in range(4)
    ]
    
    diagonals = [
        np.linalg.norm(transformed_corners[0] - transformed_corners[2]),
        np.linalg.norm(transformed_corners[1] - transformed_corners[3])
    ]
    
    theoretical_diagonal = theoretical_size * np.sqrt(2)
    
    # Print results
    print("\nTransformation Test Results:")
    print(f"Theoretical edge length: {theoretical_size:.3f} mm")
    print(f"Theoretical diagonal length: {theoretical_diagonal:.3f} mm")
    
    print("\nTransformed edge lengths:")
    for i, length in enumerate(edges, 1):
        error = abs(length - theoretical_size)
        print(f"Edge {i}: {length:.3f} mm (error: {error:.3f} mm)")
    
    print("\nTransformed diagonal lengths:")
    for i, length in enumerate(diagonals, 1):
        error = abs(length - theoretical_diagonal)
        print(f"Diagonal {i}: {length:.3f} mm (error: {error:.3f} mm)")
    
    # Calculate errors
    edge_errors = [abs(length - theoretical_size) for length in edges]
    diagonal_errors = [abs(length - theoretical_diagonal) for length in diagonals]
    
    print(f"\nEdge length statistics:")
    print(f"Maximum error: {max(edge_errors):.3f} mm")
    print(f"Average error: {sum(edge_errors)/len(edge_errors):.3f} mm")
    print(f"Standard deviation: {np.std(edge_errors):.3f} mm")
    
    print(f"\nDiagonal length statistics:")
    print(f"Maximum error: {max(diagonal_errors):.3f} mm")
    print(f"Average error: {sum(diagonal_errors)/len(diagonal_errors):.3f} mm")
    print(f"Standard deviation: {np.std(diagonal_errors):.3f} mm")

if __name__ == "__main__":
    visualize_transformation()
    test_transformation()
