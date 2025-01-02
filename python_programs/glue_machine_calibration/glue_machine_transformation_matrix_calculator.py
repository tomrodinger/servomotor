#!/usr/bin/env python3

from transformations import get_affine_transformation_matrix
from visualization import visualize
import json
import numpy as np
import os

if __name__ == "__main__":
    # Get path to calibration file in parent directory
    calibration_file = os.path.join('..', 'glue_machine_calibration.json')
    
    # Get and print transformation info
    M, theoretical_corners, projected_corners, transformed_corners = get_affine_transformation_matrix(calibration_file)
    print("\nTransformation Matrix:")
    print(M)

    # Plot some visualizations in a PNG file
    visualize([theoretical_corners, projected_corners, transformed_corners], ['Theoretical Corners', 'Projected Corners', 'Transformed Corners'], 'transformation_visualization.png')
    visualize([projected_corners, transformed_corners], ['Projected Corners', 'Transformed Corners'], 'transformation_visualization_without_theoretical_square.png')

    # Save transformation matrix to JSON in parent directory
    output = {
        'transformation_matrix': M.tolist(),
        'metadata': {
            'theoretical_corners': theoretical_corners.tolist(),
            'projected_corners': projected_corners.tolist(),
            'transformed_corners': transformed_corners.tolist()
        }
    }

    output_file = os.path.join('..', 'glue_machine_transformation_matrix.json')
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=4)
