import numpy as np
import matplotlib.pyplot as plt

def compute_half_ellipse_trajectory(start_point, end_point, height, move_vector, n_points, t_total):
    """
    Compute velocities and times for a half-ellipse trajectory between two points in 3D space.
    """
    start_point = np.array(start_point, dtype=float)
    end_point = np.array(end_point, dtype=float)
    move_vector = np.array(move_vector, dtype=float)
    
    # Vector from start to end
    D = end_point - start_point
    L = np.linalg.norm(D)
    if L == 0:
        raise ValueError("Start point and end point cannot be the same.")
    U = D / L  # Unit vector from start to end
    
    # Compute normal vector to the plane
    N = np.cross(D, move_vector)
    N_norm = np.linalg.norm(N)
    if N_norm == 0:
        raise ValueError("Move vector cannot be parallel to the direction vector from start to end.")
    W = N / N_norm  # Unit normal vector to the plane
    
    # V_axis lies in the plane and is perpendicular to U
    V_axis = np.cross(W, U)
    
    # Semi-major and semi-minor axes
    a = L / 2
    b = height
    
    # Parameter theta from 0 to pi
    theta = np.linspace(0, np.pi, n_points)
    
    # Compute ellipse points in plane coordinates
    u = a * (1 - np.cos(theta))  # From 0 to 2a
    v = b * np.sin(theta)
    
    # Compute points along the trajectory
    trajectory_points = []
    for ui, vi in zip(u, v):
        point = start_point + U * ui + V_axis * vi
        trajectory_points.append(point)
    
    # Compute distances between points
    distances = [np.linalg.norm(trajectory_points[i+1] - trajectory_points[i]) 
                 for i in range(len(trajectory_points)-1)]
    total_distance = sum(distances)
    
    # Compute times for each segment
    times = [(dist / total_distance) * t_total for dist in distances]
    
    # Compute velocities for each segment
    velocities = [(trajectory_points[i+1] - trajectory_points[i]) / times[i] 
                  for i in range(len(times))]
    
    # Append zero velocity and minimal time for the last point
    velocities.append(np.array([0.0, 0.0, 0.0]))
    times.append(0.00001)
    
    return velocities, times, trajectory_points

def simulate_movement(velocities, times, start_point, time_step=1/3125):
    """
    Simulate the movement based on velocities and times.
    """
    positions = []
    current_position = np.array(start_point, dtype=float)
    positions.append(current_position.copy())
    total_time = sum(times)
    current_time = 0.0
    velocity_index = 0
    time_in_velocity = 0.0
    while current_time < total_time:
        if velocity_index >= len(times):
            break  # Exit if velocity_index is out of bounds
        remaining_time_in_segment = times[velocity_index] - time_in_velocity
        if remaining_time_in_segment <= 0:
            # Move to next velocity
            velocity_index += 1
            time_in_velocity = 0.0
            continue
        dt = min(time_step, remaining_time_in_segment)
        # Update position
        current_position += velocities[velocity_index] * dt
        positions.append(current_position.copy())
        current_time += dt
        time_in_velocity += dt
    return positions

def test_scenarios():
    scenarios = [
        # Original scenarios
        {
            "start_point": [0, 0, 0],
            "end_point": [100, 0, 0],
            "height": 50,
            "move_vector": [0, 1, 0],
            "n_points": 50,
            "t_total": 2.0,
            "description": "Scenario 1: Half-ellipse from (0,0,0) to (100,0,0) with height 50 mm in YZ plane."
        },
        {
            "start_point": [0, 0, 0],
            "end_point": [100, 100, 0],
            "height": 70,
            "move_vector": [0, 0, 1],
            "n_points": 100,
            "t_total": 3.0,
            "description": "Scenario 2: Half-ellipse from (0,0,0) to (100,100,0) with height 70 mm in XY plane."
        },
        {
            "start_point": [0, 0, 0],
            "end_point": [0, 0, 100],
            "height": 30,
            "move_vector": [1, 1, 0],
            "n_points": 75,
            "t_total": 1.5,
            "description": "Scenario 3: Half-ellipse from (0,0,0) to (0,0,100) with height 30 mm in XZ plane."
        },
        # New scenarios with start point not at origin
        {
            "start_point": [50, 50, 0],
            "end_point": [150, 50, 0],
            "height": 40,
            "move_vector": [0, 0, 1],
            "n_points": 60,
            "t_total": 2.5,
            "description": "Scenario 4: Half-ellipse from (50,50,0) to (150,50,0) with height 40 mm in XY plane."
        },
        {
            "start_point": [100, 0, 100],
            "end_point": [200, 0, 200],
            "height": 60,
            "move_vector": [0, 1, 0],
            "n_points": 80,
            "t_total": 3.0,
            "description": "Scenario 5: Half-ellipse from (100,0,100) to (200,0,200) with height 60 mm in XZ plane."
        },
        {
            "start_point": [-50, -50, -50],
            "end_point": [50, -50, -50],
            "height": 25,
            "move_vector": [0, 1, 1],
            "n_points": 40,
            "t_total": 1.8,
            "description": "Scenario 6: Half-ellipse from (-50,-50,-50) to (50,-50,-50) with height 25 mm in an inclined plane."
        },
        {
            "start_point": [0, 100, 0],
            "end_point": [0, 200, 0],
            "height": 80,
            "move_vector": [1, 0, 0],
            "n_points": 90,
            "t_total": 2.2,
            "description": "Scenario 7: Half-ellipse from (0,100,0) to (0,200,0) with height 80 mm in YZ plane."
        },
        {
            "start_point": [25, 75, 50],
            "end_point": [125, 75, 150],
            "height": 55,
            "move_vector": [0, -1, 0],
            "n_points": 85,
            "t_total": 2.7,
            "description": "Scenario 8: Half-ellipse from (25,75,50) to (125,75,150) with height 55 mm in a custom plane."
        },
    ]
    
    for i, scenario in enumerate(scenarios):
        velocities, times, trajectory_points = compute_half_ellipse_trajectory(
            scenario["start_point"],
            scenario["end_point"],
            scenario["height"],
            scenario["move_vector"],
            scenario["n_points"],
            scenario["t_total"]
        )
        
        positions = simulate_movement(velocities, times, scenario["start_point"])
        positions = np.array(positions)
        
        # Find midpoint index
        midpoint_index = len(positions) // 2
        midpoint = positions[midpoint_index]
        
        # Plotting
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(positions[:,0], positions[:,1], positions[:,2], label='Simulated Path')
        ax.scatter(positions[0,0], positions[0,1], positions[0,2], color='green', label='Start')
        ax.scatter(positions[-1,0], positions[-1,1], positions[-1,2], color='red', label='End')
        ax.scatter(midpoint[0], midpoint[1], midpoint[2], color='blue', label='Midpoint')
        ax.set_title(scenario["description"])
        ax.legend()
        
        # Draw dashed lines to XY plane (z=0)
        # Start point
        ax.plot([positions[0,0], positions[0,0]], [positions[0,1], positions[0,1]], [positions[0,2], 0], 'g--')
        # End point
        ax.plot([positions[-1,0], positions[-1,0]], [positions[-1,1], positions[-1,1]], [positions[-1,2], 0], 'r--')
        # Midpoint
        ax.plot([midpoint[0], midpoint[0]], [midpoint[1], midpoint[1]], [midpoint[2], 0], 'b--')
        
        # Auto scaling axes
        max_range = np.array([positions[:,0].max()-positions[:,0].min(),
                              positions[:,1].max()-positions[:,1].min(),
                              positions[:,2].max()-positions[:,2].min()]).max() / 2.0

        mid_x = (positions[:,0].max()+positions[:,0].min()) * 0.5
        mid_y = (positions[:,1].max()+positions[:,1].min()) * 0.5
        mid_z = (positions[:,2].max()+positions[:,2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.savefig(f"scenario_{i+1}.png")
        plt.close()
        
if __name__ == "__main__":
    test_scenarios()
