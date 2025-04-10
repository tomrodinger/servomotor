# Glue Machine Rotation Matrix

## Overview
This project calculates a transformation matrix that maps the glue machine's coordinate system to match an object that may be slightly rotated, scaled, or translated from the machine's coordinate system. This enables accurate glue dispensing even when the target object is not perfectly aligned with the machine's axes.

## Components

### 1. Main Program (`glue_machine_rotation_matrix.py`)
Calculates the transformation matrix through the following steps:
1. Loads calibration data from `glue_machine_calibration.json`
2. Finds best-fit plane through Z-reference points using SVD
3. Projects corner points onto this plane
4. Calculates transformation matrix to map actual corners to theoretical square
5. Saves results to `glue_machine_rotation_matrix.json`

### 2. Test Program (`test_glue_machine_rotation_matrix.py`)
Provides visualization and validation:
1. Creates 3D visualization showing:
   - Actual object position (blue)
   - Z-reference points (red)
   - Theoretical square (green)
   - Transformed square (orange)
2. Tests transformation accuracy by:
   - Applying transformation to corner points
   - Measuring transformed edge lengths
   - Comparing with theoretical size
   - Reporting maximum and average errors

## Algorithm Details

### Best-Fit Plane Calculation
- Uses Singular Value Decomposition (SVD) on centered points
- Normal vector is the third right singular vector
- Provides optimal plane minimizing squared distances to points

### Point Projection
Projects corner points onto best-fit plane:
```python
projected_point = point - dot(point - plane_point, normal) * normal
```

### Transformation Matrix Calculation
1. Centers both actual and theoretical points at origin
2. Uses SVD to find optimal rotation matrix
3. Calculates scale factor from average point distances
4. Combines translation, rotation, and scaling into 4x4 matrix

## File Formats

### Input (`glue_machine_calibration.json`)
```json
{
    "corner1": {"x": float, "y": float, "z": float},
    "corner2": {"x": float, "y": float, "z": float},
    "corner3": {"x": float, "y": float, "z": float},
    "corner4": {"x": float, "y": float, "z": float},
    "zref1": {"x": float, "y": float, "z": float},
    "zref2": {"x": float, "y": float, "z": float},
    "zref3": {"x": float, "y": float, "z": float},
    "zref4": {"x": float, "y": float, "z": float}
}
```

### Output (`glue_machine_rotation_matrix.json`)
```json
{
    "transform_matrix": [
        [float, float, float, float],
        [float, float, float, float],
        [float, float, float, float],
        [float, float, float, float]
    ],
    "metadata": {
        "theoretical_square_size": float,
        "plane_normal": [float, float, float],
        "plane_point": [float, float, float]
    }
}
```

## Usage

1. Ensure calibration data exists:
```bash
python3 calibrate_glue_machine.py
```

2. Calculate transformation matrix:
```bash
python3 glue_machine_rotation_matrix.py
```

3. Test and visualize results:
```bash
python3 test_glue_machine_rotation_matrix.py
```

## Dependencies
- NumPy: Matrix operations and linear algebra
- SciPy: SVD calculation
- Matplotlib: 3D visualization
- JSON: Data file handling

## Notes
- Theoretical square size is set to 298.80108mm
- Transformation preserves relative distances and angles
- Visualization shows all coordinate systems for verification
- Error metrics help validate transformation accuracy
