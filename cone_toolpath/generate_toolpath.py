#!/usr/bin/env python3
"""Script to generate a 6-axis robot toolpath along a cone surface."""

import os
import csv
import numpy as np
from scipy.optimize import minimize

# Parameters for the Cone
H = 1.0     # Height of the cone
R = 0.5     # Base radius of the cone
N_POINTS = 200  # Number of points in the path
TURNS = 8       # Number of spiral turns

def generate_cone_model(filename):
    """Generate a wavefront OBJ file of the cone model for reference."""
    vertices = []
    faces = []

    # Apex vertex (index 1)
    vertices.append((0.0, 0.0, H))
    
    # Base center vertex (index 2)
    vertices.append((0.0, 0.0, 0.0))

    # Base circle vertices (indices 3 to 3+n-1)
    n_base = 32
    for i in range(n_base):
        theta = 2.0 * np.pi * i / n_base
        x = R * np.cos(theta)
        y = R * np.sin(theta)
        vertices.append((x, y, 0.0))

    # Faces connecting base to apex (sides)
    for i in range(n_base):
        v1 = 3 + i
        v2 = 3 + (i + 1) % n_base
        faces.append((1, v2, v1))

    # Faces for the base cap
    for i in range(n_base):
        v1 = 3 + i
        v2 = 3 + (i + 1) % n_base
        faces.append((2, v1, v2))

    with open(filename, 'w') as f:
        f.write("# Cone Model Reference Geometry\n")
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")

def rz(theta):
    """Z-axis rotation matrix."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def ry(theta):
    """Y-axis rotation matrix."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def rx(theta):
    """X-axis rotation matrix."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def tz(d):
    """Z-axis translation matrix."""
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

# Robot Link Lengths
L1 = 0.4   # Base height
L2 = 0.45  # Upper arm
L3 = 0.4   # Forearm
L4 = 0.1   # Wrist extension
L5 = 0.1   # Wrist pitch
L6 = 0.05  # Tool length

def forward_kinematics(q):
    """Compute forward kinematics for a 6-axis articulated robot."""
    T01 = rz(q[0]) @ tz(L1)
    T12 = ry(q[1]) @ tz(L2)
    T23 = ry(q[2]) @ tz(L3)
    T34 = rx(q[3]) @ tz(L4)
    T45 = ry(q[4]) @ tz(L5)
    T56 = rz(q[5]) @ tz(L6)
    
    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    return T

def ik_loss(q, T_target, q_prev):
    """Calculate the loss function for numerical inverse kinematics."""
    T_curr = forward_kinematics(q)
    
    # Position error
    pos_err = np.sum((T_curr[:3, 3] - T_target[:3, 3])**2)
    
    # Orientation error (Frobenius norm of rotation matrix difference)
    rot_err = np.sum((T_curr[:3, :3] - T_target[:3, :3])**2)
    
    # Joint configuration change penalty (maintains path smoothness)
    smooth_err = np.sum((q - q_prev)**2)
    
    return pos_err * 100.0 + rot_err * 10.0 + smooth_err * 0.1

def solve_ik(T_target, q_init):
    """Solve inverse kinematics using numerical optimization."""
    # Joint limits (in radians)
    bounds = [
        (-np.pi, np.pi),      # Joint 1
        (-np.pi/2, np.pi/2),  # Joint 2
        (-np.pi, np.pi),      # Joint 3
        (-np.pi, np.pi),      # Joint 4
        (-np.pi, np.pi),      # Joint 5
        (-np.pi, np.pi)       # Joint 6
    ]
    res = minimize(
        ik_loss, 
        q_init, 
        args=(T_target, q_init), 
        method='SLSQP', 
        bounds=bounds,
        tol=1e-6
    )
    return res.x

def main():
    print("Generating cone model geometry...")
    os.makedirs("cone_toolpath", exist_ok=True)
    generate_cone_model("cone_toolpath/cone_model.obj")
    
    # Time vector (duration: 20 seconds)
    total_time = 20.0
    times = np.linspace(0, total_time, N_POINTS)
    
    csv_rows = []
    q_prev = np.array([0.0, 0.1, 0.2, 0.0, 0.1, 0.0]) # Initial joint angles guess

    print("Generating spiral toolpath and computing joint trajectories...")
    for i, t in enumerate(times):
        # Parametrize spiral from top to bottom
        progress = t / total_time
        z = H * (1.0 - progress)
        r = R * progress
        theta = 2.0 * np.pi * TURNS * progress
        
        # Position on the cone surface
        x_cone = r * np.cos(theta)
        y_cone = r * np.sin(theta)
        
        # 3D Position offset for robot reachability
        # (Shift cone to be in front of the robot at X=1.0)
        p_target = np.array([1.0 + x_cone, y_cone, z])
        
        # Compute local tangent and normal vectors for orientation
        # Surface normal (outward)
        alpha = np.arctan2(R, H)  # Semi-vertical angle
        nx = np.cos(theta) * np.cos(alpha)
        ny = np.sin(theta) * np.cos(alpha)
        nz = np.sin(alpha)
        
        # Tool Z-axis points inwards normal to cone surface
        z_tool = -np.array([nx, ny, nz])
        z_tool /= np.linalg.norm(z_tool)
        
        # Tangent vector along spiral path
        dx = np.cos(theta) * (R/total_time) - r * np.sin(theta) * (2*np.pi*TURNS/total_time)
        dy = np.sin(theta) * (R/total_time) + r * np.cos(theta) * (2*np.pi*TURNS/total_time)
        dz = -H/total_time
        y_tool = np.array([dx, dy, dz])
        y_tool /= np.linalg.norm(y_tool)
        
        # X-axis completes right-handed coordinate frame
        x_tool = np.cross(y_tool, z_tool)
        x_tool /= np.linalg.norm(x_tool)
        
        # Re-orthogonalize Y-axis
        y_tool = np.cross(z_tool, x_tool)
        
        # Target transform matrix
        T_target = np.eye(4)
        T_target[:3, 0] = x_tool
        T_target[:3, 1] = y_tool
        T_target[:3, 2] = z_tool
        T_target[:3, 3] = p_target
        
        # Solve Inverse Kinematics
        q_sol = solve_ik(T_target, q_prev)
        q_prev = q_sol
        
        # Save values (Time, Joint1, ..., Joint6)
        row = [f"{t:.2f}"] + [f"{q:.4f}" for q in q_sol]
        csv_rows.append(row)

    csv_path = "cone_toolpath/cone_toolpath.csv"
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"])
        writer.writerows(csv_rows)
        
    print(f"Toolpath successfully written to {csv_path}")

    # Optional plot visualization
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot cone surface
        cone_z = np.linspace(0, H, 50)
        cone_theta = np.linspace(0, 2*np.pi, 50)
        cone_z_grid, cone_theta_grid = np.meshgrid(cone_z, cone_theta)
        cone_r_grid = R * (cone_z_grid / H)
        cone_x = cone_r_grid * np.cos(cone_theta_grid) + 1.0
        cone_y = cone_r_grid * np.sin(cone_theta_grid)
        ax.plot_surface(cone_x, cone_y, H - cone_z_grid, alpha=0.3, color='cyan')
        
        # Plot spiral toolpath
        csv_data = np.array(csv_rows, dtype=float)
        traj_points = []
        for row in csv_data:
            T = forward_kinematics(row[1:])
            traj_points.append(T[:3, 3])
        traj_points = np.array(traj_points)
        
        ax.plot(traj_points[:, 0], traj_points[:, 1], traj_points[:, 2], 'r-', linewidth=2, label='Robot Toolpath')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('6-Axis Robot Toolpath on Cone Surface')
        ax.legend()
        plt.savefig("cone_toolpath/toolpath_visualization.png")
        print("Visualization saved to cone_toolpath/toolpath_visualization.png")
    except Exception as e:
        print(f"Visualization plot skipped: {e}")

if __name__ == '__main__':
    main()
