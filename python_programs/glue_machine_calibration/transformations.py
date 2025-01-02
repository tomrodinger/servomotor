import numpy as np
from scipy.linalg import svd
from .data_utils import load_calibration_data, points_to_array

# Create theoretical square
EDGE_LENGTH = 61.0 * 3
THEORETICAL_SQUARE_DIAGONAL_LENGTH = EDGE_LENGTH * np.sqrt(2)
HALF_EDGE_LENGTH = EDGE_LENGTH / 2

THEORETICAL_CORNERS = np.array([
#        [0, 0, 0],
#        [EDGE_LENGTH, EDGE_LENGTH, 0],
#        [0, EDGE_LENGTH, 0],
#        [EDGE_LENGTH, 0, 0]
    [-HALF_EDGE_LENGTH, -HALF_EDGE_LENGTH, 0],
    [HALF_EDGE_LENGTH, HALF_EDGE_LENGTH, 0],
    [-HALF_EDGE_LENGTH, HALF_EDGE_LENGTH, 0],
    [HALF_EDGE_LENGTH, -HALF_EDGE_LENGTH, 0]
])

def fit_plane(points):
    """Find best-fit plane through points using SVD."""
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = svd(centered_points)
    normal = vh[2]  # Third row of vh is normal to the plane
    # Ensure normal points upward (positive z)
    if normal[2] < 0:
        normal = -normal
    return normal, centroid

def project_point_to_plane(point, plane_normal, plane_point):
    """Project a point onto a plane defined by normal vector and a point on the plane."""
    v = point - plane_point
    dist = np.dot(v, plane_normal)
    projected_point = point - dist * plane_normal
    return projected_point

def get_rotation_to_xy(points):
    """Get rotation matrix that aligns points with Z=0 plane using SVD."""
    # Get the best-fit plane normal
    _, _, vh = svd(points)
    normal = vh[2]
    if normal[2] < 0:
        normal = -normal

    # Target normal (Z axis)
    target = np.array([0, 0, 1])

    # Create rotation matrix using Rodrigues' rotation formula
    v = np.cross(normal, target)
    s = np.linalg.norm(v)
    c = np.dot(normal, target)

    if abs(s) < 1e-10:  # Vectors are parallel
        return np.eye(3)

    v_skew = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

    R = np.eye(3) + v_skew + np.dot(v_skew, v_skew) * (1 - c) / (s * s)
    return R.T

def umeyama_alignment(source_points, target_points):
    """
    Estimate N-dimensional similarity transformation (rotation, translation, scaling) that maps source_points to target_points.
    """
    source_points = np.asarray(source_points)
    target_points = np.asarray(target_points)
    assert source_points.shape == target_points.shape

    N, dim = source_points.shape

    # Compute centroids
    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)

    # Center the points
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean

    # Compute variance of the source points
    source_var = np.sum(source_centered ** 2) / N

    # Compute covariance matrix
    covariance = np.dot(target_centered.T, source_centered) / N

    # Singular Value Decomposition
    U, D, Vt = np.linalg.svd(covariance)

    # Compute rotation matrix
    R = np.dot(U, Vt)

    # Correct for reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(U, Vt)

    # Compute scale factor
    scale = np.trace(np.dot(np.diag(D), np.eye(dim))) / source_var

    # Compute translation
    t = target_mean - scale * np.dot(R, source_mean)

    # Construct transformation matrix
    T = np.eye(dim + 1)
    T[:dim, :dim] = scale * R
    T[:dim, dim] = t

    return T, scale, R, t

def get_transformation_matrix(calibration_file):
    """Calculate the complete transformation matrix using Umeyama method."""
    # Load data
    data = load_calibration_data(calibration_file)
    corner_points = points_to_array(data, ['corner1', 'corner2', 'corner3', 'corner4'])
    zref_points = points_to_array(data, ['zref1', 'zref2', 'zref3', 'zref4'])
        
    # Fit plane to zref_points and project corner_points onto it
    plane_normal, plane_point = fit_plane(zref_points)
    projected_corners = np.array([
        project_point_to_plane(corner, plane_normal, plane_point)
        for corner in corner_points
    ])
    
    # Apply Umeyama alignment
    T_umeyama, scale, R_umeyama, t_umeyama = umeyama_alignment(THEORETICAL_CORNERS, projected_corners)
    
    # Print transformation details
    print("\nScale factor from Umeyama method:", scale)
    print("\nRotation matrix from Umeyama method:\n", R_umeyama)
    print("\nTranslation vector from Umeyama method:\n", t_umeyama)
    
    # Apply transformation to theoretical points
    theoretical_corners_h = np.hstack([THEORETICAL_CORNERS, np.ones((THEORETICAL_CORNERS.shape[0], 1))])
    transformed_corners = (T_umeyama @ theoretical_corners_h.T).T[:, :3]
    
    # Compute distances between transformed and projected points
    distances = np.linalg.norm(transformed_corners - projected_corners, axis=1)
    print("\nDistances between transformed theoretical points and projected points (in mm):")
    for i, dist in enumerate(distances):
        print(f"Point {i+1}: {dist:.6f} mm")
    
    return T_umeyama, THEORETICAL_CORNERS, projected_corners, transformed_corners


def get_affine_transformation_matrix_z_scaling_0(calibration_file):
    """Calculate the affine transformation matrix with anisotropic scaling."""
    # Load data
    data = load_calibration_data(calibration_file)
    corner_points = points_to_array(data, ['corner1', 'corner2', 'corner3', 'corner4'])
    zref_points = points_to_array(data, ['zref1', 'zref2', 'zref3', 'zref4'])
        
    # Project corner points onto the plane defined by zref_points
    plane_normal, plane_point = fit_plane(zref_points)
    projected_corners = np.array([
        project_point_to_plane(corner, plane_normal, plane_point)
        for corner in corner_points
    ])
    
    # Prepare matrices
    N = THEORETICAL_CORNERS.shape[0]
    X = THEORETICAL_CORNERS  # Source points
    Y = projected_corners    # Target points
    
    # Augment X with ones for affine transformation
    X_aug = np.hstack([X, np.ones((N, 1))])  # Shape (N, 4)
    
    # Solve for affine transformation matrix using least squares
    # T_aug will be a (4, 3) matrix
    T_aug, residuals, rank, s = np.linalg.lstsq(X_aug, Y, rcond=None)
    
    # Extract A and t
    A = T_aug[:3, :].T  # Shape (3, 3)
    t = T_aug[3, :]     # Shape (3,)
    
    # Assemble the full 4x4 transformation matrix
    M = np.eye(4)
    M[:3, :3] = A
    M[:3, 3] = t
    
    # Apply transformation to theoretical points
    theoretical_corners_h = np.hstack([THEORETICAL_CORNERS, np.ones((N, 1))])
    transformed_corners = (M @ theoretical_corners_h.T).T[:, :3]
    
    # Print out the transformed points and the projected points and the differnces between them (in X, Y, and Z directions)
    print("\nTransformed theoretical points:")
    print(transformed_corners)
    print("\nProjected points:")
    print(projected_corners)
    print("\nDifferences between transformed and projected points (in mm):")
    differences = transformed_corners - projected_corners
    for i, diff in enumerate(differences):
        print(f"Point {i+1}: {diff}")
    
    # Compute distances between transformed theoretical points and projected points
    distances = np.linalg.norm(transformed_corners - projected_corners, axis=1)
    print("\nDistances between transformed theoretical points and projected points (in mm):")
    for i, dist in enumerate(distances):
        print(f"Point {i+1}: {dist:.6f} mm")
    
    # Extract scaling factors from A
    # Singular Value Decomposition to extract scaling and rotation
    U, S_values, Vt = np.linalg.svd(A)
    S = np.diag(S_values)
    R = U @ Vt
    scales = S_values  # Scaling factors along principal axes
    
    print("\nScaling factors along principal axes:")
    print(f"Scale X: {scales[0]:.6f}")
    print(f"Scale Y: {scales[1]:.6f}")
    print(f"Scale Z: {scales[2]:.6f}")
    
    # Since A includes scaling and rotation, we can separate them if needed
    # However, the scaling factors may not correspond directly to X, Y, Z axes if rotation is involved
    
    return M, THEORETICAL_CORNERS, projected_corners, transformed_corners


def get_affine_transformation_matrix(calibration_file):
    """Calculate the affine transformation matrix with scaling along Z fixed to -1.0."""
    # Load data
    data = load_calibration_data(calibration_file)
    corner_points = points_to_array(data, ['corner1', 'corner2', 'corner3', 'corner4'])
    zref_points = points_to_array(data, ['zref1', 'zref2', 'zref3', 'zref4'])
        
    # Project corner points onto the plane defined by zref_points
    plane_normal, plane_point = fit_plane(zref_points)
    projected_corners = np.array([
        project_point_to_plane(corner, plane_normal, plane_point)
        for corner in corner_points
    ])
    
    N = THEORETICAL_CORNERS.shape[0]
    X = THEORETICAL_CORNERS  # Source points
    Y = projected_corners    # Target points
    
    # Augment X with ones for affine transformation
    X_aug = np.hstack([X, np.ones((N, 1))])  # Shape (N, 4)
    
    # Solve for affine transformation matrix using least squares
    # T_aug will be a (4, 3) matrix
    T_aug, residuals, rank, s = np.linalg.lstsq(X_aug, Y, rcond=None)
    
    # Extract A and t
    A = T_aug[:3, :].T  # Shape (3, 3)
    t = T_aug[3, :]     # Shape (3,)
    
    # Perform SVD on A
    U, S_values, Vt = np.linalg.svd(A)
    
    print("\nOriginal scaling factors along principal axes:")
    print(f"Scale X: {S_values[0]:.6f}")
    print(f"Scale Y: {S_values[1]:.6f}")
    print(f"Scale Z: {S_values[2]:.6f}")
    
    # Adjust scaling factors by setting the scaling along Z to -1.0 (-1 will reverse the Z axis direction)
    S_values[2] = -1.0
    S_corrected = np.diag(S_values)
    
    # Reconstruct the adjusted A matrix
    A_corrected = U @ S_corrected @ Vt
    
    # Reassemble the full 4x4 transformation matrix
    M = np.eye(4)
    M[:3, :3] = A_corrected
    M[:3, 3] = t
    
    # Apply the corrected transformation to theoretical points
    theoretical_corners_h = np.hstack([THEORETICAL_CORNERS, np.ones((N, 1))])
    transformed_corners = (M @ theoretical_corners_h.T).T[:, :3]
    
    # Compute differences between transformed and projected points
    differences = transformed_corners - projected_corners
    distances = np.linalg.norm(differences, axis=1)
    
    print("\nAdjusted scaling factors along principal axes:")
    print(f"Scale X: {S_values[0]:.6f}")
    print(f"Scale Y: {S_values[1]:.6f}")
    print(f"Scale Z: {S_values[2]:.6f}")
    
    # Print out the transformed points and the projected points and the differences between them
    print("\nTransformed theoretical points:")
    print(transformed_corners)
    print("\nProjected points:")
    print(projected_corners)
    print("\nDifferences between transformed and projected points (in mm):")
    for i, diff in enumerate(differences):
        print(f"Point {i+1}: {diff}")
    
    # Compute distances between transformed theoretical points and projected points
    print("\nDistances between transformed theoretical points and projected points (in mm):")
    for i, dist in enumerate(distances):
        print(f"Point {i+1}: {dist:.6f} mm")
    
    return M, THEORETICAL_CORNERS, projected_corners, transformed_corners
