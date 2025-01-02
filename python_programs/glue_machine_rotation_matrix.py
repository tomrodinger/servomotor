import numpy as np
import json
from scipy.linalg import svd

def load_calibration_data(filename):
    """Load calibration data from JSON file."""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def points_to_array(data, point_names):
    """Convert points from dictionary to numpy array."""
    points = []
    for name in point_names:
        point = data[name]
        points.append([point['x'], point['y'], point['z']])
    return np.array(points)

def fit_plane(points):
    """
    Find best-fit plane through points using SVD.
    Returns normal vector and point on plane (centroid).
    """
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = svd(centered_points)
    normal = vh[2]  # Third row of vh is normal to the plane
    # Ensure normal points upward (positive z)
    if normal[2] < 0:
        normal = -normal
    return normal, centroid

def get_rotation_to_xy(normal):
    """Get rotation matrix that aligns normal vector with Z axis."""
    z_axis = np.array([0, 0, 1])
    v = np.cross(normal, z_axis)
    s = np.linalg.norm(v)
    
    if s < 1e-10:  # Normal already aligned with Z axis
        return np.eye(3)
    
    c = np.dot(normal, z_axis)
    v_skew = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
    
    R = np.eye(3) + v_skew + np.dot(v_skew, v_skew) * (1 - c) / (s * s)
    return R

def print_debug_info(points, label=""):
    """Print debug information about points."""
    print(f"\nDebug info for {label}:")
    edges = [
        np.linalg.norm(points[3] - points[0]),  # Bottom edge
        np.linalg.norm(points[1] - points[2]),  # Top edge
        np.linalg.norm(points[2] - points[0]),  # Left edge
        np.linalg.norm(points[1] - points[3])   # Right edge
    ]
    print(f"Edge lengths: {edges}")
    
    diagonals = [
        np.linalg.norm(points[2] - points[0]),  # Left diagonal
        np.linalg.norm(points[3] - points[1])   # Right diagonal
    ]
    print(f"Diagonal lengths: {diagonals}")

def calculate_transformation_matrix(actual_corners, theoretical_size):
    """
    Calculate transformation matrix from actual corners to theoretical square.
    theoretical_size: edge length of the theoretical square
    
    Corner order:
    actual_corners[0] = corner1 (10, 9)    - bottom left
    actual_corners[1] = corner2 (221, 220) - top right
    actual_corners[2] = corner3 (10, 220)  - top left
    actual_corners[3] = corner4 (221, 9)   - bottom right
    """
    print_debug_info(actual_corners, "Original corners")
    
    # Calculate centroid and normal
    centroid = np.mean(actual_corners, axis=0)
    centered_points = actual_corners - centroid
    _, _, vh = svd(centered_points)
    normal = vh[2]
    if normal[2] < 0:
        normal = -normal
    
    # Get rotation to align with XY plane
    R_to_xy = get_rotation_to_xy(normal)
    
    # Apply rotation to points
    rotated_points = np.dot(centered_points, R_to_xy.T)
    print_debug_info(rotated_points, "After XY alignment")
    
    # Calculate angle in XY plane using bottom edge
    bottom_edge = rotated_points[3] - rotated_points[0]  # corner4 - corner1
    angle = np.arctan2(bottom_edge[1], bottom_edge[0])
    
    # Create rotation matrix for XY plane alignment
    R_xy = np.array([
        [np.cos(-angle), -np.sin(-angle), 0],
        [np.sin(-angle), np.cos(-angle), 0],
        [0, 0, 1]
    ])
    
    # Apply XY rotation
    aligned_points = np.dot(rotated_points, R_xy.T)
    print_debug_info(aligned_points, "After angle alignment")
    
    # Create theoretical square corners to match actual corner ordering
    half_size = theoretical_size / 2
    theoretical_corners = np.array([
        [-half_size, -half_size, 0],  # Bottom left  (matches corner1)
        [half_size, half_size, 0],    # Top right    (matches corner2)
        [-half_size, half_size, 0],   # Top left     (matches corner3)
        [half_size, -half_size, 0]    # Bottom right (matches corner4)
    ])
    
    # Calculate scale factor using parallel edges
    bottom_edge_length = np.linalg.norm(aligned_points[3] - aligned_points[0])
    top_edge_length = np.linalg.norm(aligned_points[1] - aligned_points[2])
    left_edge_length = np.linalg.norm(aligned_points[2] - aligned_points[0])
    right_edge_length = np.linalg.norm(aligned_points[1] - aligned_points[3])
    
    avg_width = (bottom_edge_length + top_edge_length) / 2
    avg_height = (left_edge_length + right_edge_length) / 2
    scale = theoretical_size / np.mean([avg_width, avg_height])
    
    # Create complete transformation matrix
    transform = np.eye(4)
    R_combined = np.dot(R_xy, R_to_xy)
    transform[:3, :3] = scale * R_combined
    transform[:3, 3] = -np.dot(scale * R_combined, centroid)
    
    return transform, aligned_points

def main():
    # Load calibration data
    data = load_calibration_data('glue_machine_calibration.json')
    
    # Get corner points
    corner_points = points_to_array(data, ['corner1', 'corner2', 'corner3', 'corner4'])
    
    # Calculate transformation matrix
    theoretical_size = 298.80108  # mm (given in task)
    transform_matrix, aligned_corners = calculate_transformation_matrix(corner_points, theoretical_size)
    
    # Save transformation matrix to JSON
    output = {
        'transform_matrix': transform_matrix.tolist(),
        'metadata': {
            'theoretical_square_size': theoretical_size,
            'aligned_corners': aligned_corners.tolist()
        }
    }
    
    with open('glue_machine_rotation_matrix.json', 'w') as f:
        json.dump(output, f, indent=4)

if __name__ == "__main__":
    main()
