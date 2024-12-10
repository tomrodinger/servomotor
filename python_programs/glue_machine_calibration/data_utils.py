# data_utils.py

import json
import numpy as np

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
