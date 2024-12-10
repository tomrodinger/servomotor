#!/usr/bin/env python3

from transformations import get_affine_transformation_matrix
from visualization import visualize
import json
import numpy as np

if __name__ == "__main__":
    # Get and print transformation info
    M, theoretical_corners, projected_corners, transformed_corners = get_affine_transformation_matrix()
    print("\nTransformation Matrix:")
    print(M)

    # Plot some visualizations in a PNG file
    visualize([theoretical_corners, projected_corners, transformed_corners], ['Theoretical Corners', 'Projected Corners', 'Transformed Corners'], 'transformation_visualization.png')
    visualize([projected_corners, transformed_corners], ['Projected Corners', 'Transformed Corners'], 'transformation_visualization_without_theoretical_square.png')

    # Save transformation matrix to JSON
    output = {
        'transformation_matrix': M.tolist(),
        'metadata': {
            'theoretical_corners': theoretical_corners.tolist(),
            'projected_corners': projected_corners.tolist(),
            'transformed_corners': transformed_corners.tolist()
        }
    }

    with open('glue_machine_transformation_matrix.json', 'w') as f:
        json.dump(output, f, indent=4)
