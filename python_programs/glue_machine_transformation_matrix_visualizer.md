# Glue Machine Transformation Matrix Visualizer

## Project Purpose
This project is part of a glue dispensing machine calibration system. It aims to find a transformation matrix that can convert between:
- Theoretical square points (in an ideal coordinate system) that the machine is controlling
- Actual coordinates where glue needs to be applied

The transformation needs to account for any rotation, translation, and scaling between these coordinate systems.

## Current State
The project consists of two main files:
1. `glue_machine_transformation_matrix_visualizer.py`: The main program
2. `glue_machine_calibration.json`: Contains calibration points

The program currently:
- Loads calibration points
- Creates a theoretical square (211.284mm edge length, 298.80108mm diagonal)
- Attempts to find a transformation matrix
- Visualizes the points and transformations

## Data Points
The program works with several sets of points:
1. **Theoretical Square Points**:
   - theor1: (0, 0, 0)
   - theor2: (211.284, 211.284, 0)
   - theor3: (0, 211.284, 0)
   - theor4: (211.284, 0, 0)

2. **Calibration Points** (from JSON file):
   - corner1, corner2, corner3, corner4: The actual corner measurements
   - zref1, zref2, zref3, zref4: Z-reference points defining the plane

3. **Projected Points**:
   - Corner points projected onto the plane defined by zref points

## Current Implementation
The transformation process attempts to:
1. Project corner points onto the best-fit plane from zref points
2. Find a transformation matrix that maps between theoretical and projected points
3. Visualize the results showing:
   - Original corner points (blue)
   - Projected corner points (green)
   - Perfect square points (purple)

## Issues to Address
1. Need to verify Z coordinates after rotation step
2. Need to verify distances between points after scaling step
3. Current transformation may need to be reversed (from projected to theoretical instead of theoretical to projected)

## Next Steps
1. Add print statements to show:
   - Z coordinates after rotation
   - Distances between points after scaling
2. Verify the transformation order:
   - Currently: T2 * R.T * S * T1
   - May need to transform projected points to match theoretical points instead
3. Ensure scaling factors are applied in the correct direction

## Visualization
The program creates two visualizations:
1. `transformation_final.png`: Shows all points (theoretical, transformed, and projected)
2. `transformation_final_without_theoretical_points.png`: Shows only transformed and projected points

## Output Files
1. `glue_machine_transformation_matrix.json`: Contains the final transformation matrix and metadata
   ```json
   {
       "transformation_matrix": [...],
       "metadata": {
           "theoretical_square_diagonal": 298.80108,
           "theoretical_square_edge": 211.284
       }
   }
   ```

## Dependencies
- NumPy: For numerical computations
- Matplotlib: For 3D visualization
- SciPy: For SVD calculations

## Key Functions
1. `get_rotation_to_xy(points)`: Calculates rotation matrix to align points with Z=0 plane
2. `fit_plane(points)`: Finds best-fit plane through points using SVD
3. `project_point_to_plane(point, plane_normal, plane_point)`: Projects points onto plane
4. `get_transformation_matrix()`: Calculates the complete transformation matrix
5. `plot_transformation(include_theoretical)`: Creates visualizations

## Future Improvements
1. Add detailed logging of intermediate steps
2. Verify rotation aligns points properly with Z=0
3. Ensure scaling preserves square properties
4. Add validation of transformation accuracy
5. Consider adding inverse transformation calculation

## Related Files
- `calibrate_glue_machine.py`: Original calibration program
- `calibrate_glue_machine.md`: Documentation for calibration process

## Notes
- The theoretical square diagonal (298.80108mm) is a critical reference measurement
- The transformation should maintain square properties (equal edges, right angles)
- Z coordinates after rotation should be very close to 0
- Final transformed points should closely match projected points

# Mathematical Theory

## Transformation Pipeline
The transformation between coordinate systems involves several steps:

1. **Translation**
   - Centers points around origin
   - Uses homogeneous coordinates for 4x4 matrix operations
   - Translation matrix T = [I₃ t; 0 1] where t is translation vector

2. **Rotation**
   - Aligns points with Z=0 plane
   - Uses SVD to find plane normal
   - Rodrigues' rotation formula creates rotation matrix:
     ```
     R = I + [v]ₓ + [v]ₓ²(1-cos(θ))/sin²(θ)
     where [v]ₓ is the skew-symmetric matrix of v
     ```

3. **Scaling**
   - Adjusts for any size differences
   - Separate X and Y scaling factors
   - Preserves Z coordinates (no Z scaling)

## Coordinate Systems
1. **Theoretical Space**
   - Perfect square at origin
   - Z=0 plane
   - Known edge length (211.284mm)
   - Known diagonal (298.80108mm)

2. **Machine Space**
   - Actual measured coordinates
   - Points may not form perfect square
   - Points may not lie in Z=0 plane

3. **Projected Space**
   - Machine points projected onto best-fit plane
   - Maintains machine X,Y positions
   - Adjusts Z to lie in plane

# Calibration Process

## Physical Setup
1. Place calibration object (square) in machine
2. Object should be:
   - Roughly aligned with machine axes
   - Within machine's working volume
   - On a stable surface

## Measurement Steps
1. **Z-Reference Points**
   - Measure 4 points defining the working plane
   - Points should form roughly rectangular pattern
   - Used to calculate plane normal

2. **Corner Points**
   - Measure actual corner positions
   - Order matters: corner1->corner3->corner2->corner4
   - Forms basis for transformation

## Validation
1. Project corner points onto Z-ref plane
2. Calculate transformation matrix
3. Verify:
   - Z coordinates near zero after rotation
   - Square properties preserved
   - Small distances between corresponding points

# Glue Dispensing Integration

## Machine Operation
1. **Calibration Phase**
   - Run once per setup
   - Measures reference points
   - Calculates transformation matrix

2. **Production Phase**
   - Loads transformation matrix
   - Converts theoretical coordinates to machine coordinates
   - Applies glue at transformed positions

## Coordinate Flow
1. **Design Software**
   - Outputs theoretical glue coordinates
   - Assumes perfect square alignment
   - Uses Z=0 plane

2. **Transformation**
   - Applies stored matrix to coordinates
   - Accounts for:
     - Object position
     - Object rotation
     - Surface tilt
     - Scale variations

3. **Machine Control**
   - Moves to transformed coordinates
   - Dispenses glue
   - Maintains proper distance to surface

## Error Handling
1. **Transformation Validation**
   - Check transformed points are within machine bounds
   - Verify Z coordinates are reasonable
   - Ensure square properties maintained

2. **Runtime Checks**
   - Monitor transformation results
   - Flag suspicious coordinates
   - Allow operator intervention

## Safety Considerations
1. **Mechanical**
   - Ensure transformed points within safe range
   - Check for sudden position changes
   - Maintain safe Z clearance

2. **Process**
   - Verify glue path continuity
   - Check dispensing heights
   - Monitor transformation accuracy
