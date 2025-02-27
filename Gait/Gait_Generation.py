import sys
import os

# Add the parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from Kinematics.Full_Kinematics import *
import numpy as np
import matplotlib.pyplot as plt



def plot_foot_path(stride_length, h1, h2, num_points=15):
    L = stride_length

    # Generate x values for the swing phase and compute y using an inverted parabola.
    x_swing = np.linspace(0, L, num_points)
    y_swing = -4 * h1 / L**2 * (x_swing - L/2)**2 + h1

    # Generate x values for the stance phase and compute y using a concave up parabola.
    x_stance = np.linspace(L,0, num_points)
    y_stance = 4 * h2 / L**2 * (x_stance - L/2)**2 - h2

    # Plot the two phases.
    plt.figure(figsize=(10, 4))
    plt.plot(x_swing, y_swing, 'b-', linewidth=2, label='Swing Phase (Inverted Parabola)')
    plt.plot(x_stance, y_stance, 'r-', linewidth=2, label='Stance Phase (Upright Parabola)')
    
    plt.xlabel('Horizontal Distance')
    plt.ylabel('Foot Height')
    plt.title('Quadruped Robot Foot Path')
    plt.legend()
    plt.grid(True)
    plt.show()
    print(x_swing,x_stance)
    return(x_swing,y_swing,x_stance,y_stance)

def calculate_radii_front(x_swing,y_swing,x_stance,y_stance): 
    #Calculate Inverse Kinematics for legs for each point on foot path
    leg = Leg_Kinematics(0,100,-175,[106,106],np.pi/9,"c")
    r = []
    t = []
    for i in range(len(x_swing)):
      leg.y = 100 - x_swing[i]
      leg.z = -175 + y_swing[i]
      leg.compute_lateral_bending()
      leg.compute_inplane_bending()
      leg.compute_tendon_lengths()
      r.append([leg.radius2,leg.radius3])
      t.append([leg.t[3],leg.t[5]])
    for i in range(len(x_stance)):
      leg.y = 100 - x_stance[i]
      leg.z = -175 + y_stance[i]
      leg.compute_lateral_bending()
      leg.compute_inplane_bending()
      leg.compute_tendon_lengths()
      r.append([leg.radius2,leg.radius3])
      t.append([leg.t[3],leg.t[5]])
    return(r,t)

def calculate_radii_back(x_swing,y_swing,x_stance,y_stance):
    leg = Leg_Kinematics(0,40,-175,[106,106],np.pi/9,"c")
    r = []
    t = []
    for i in range(len(x_swing)):
      leg.y = 40 + x_swing[i]
      leg.z = -175 + y_swing[i]
      leg.compute_lateral_bending()
      leg.compute_inplane_bending()
      leg.compute_tendon_lengths()
      r.append([leg.radius2,leg.radius3])
      t.append([leg.t[3],leg.t[5]])
    for i in range(len(x_stance)):
      leg.y = 40 + x_stance[i]
      leg.z = -175 + y_stance[i]
      leg.compute_lateral_bending()
      leg.compute_inplane_bending()
      leg.compute_tendon_lengths()
      r.append([leg.radius2,leg.radius3])
      t.append([leg.t[3],leg.t[5]])
    return(r,t)
    
def compute_beam_points(radii, l1, l2, num_points=400):
    r1, r2 = radii
    theta0 = -np.pi / 9  # Initial tangent: +30° (30° up from horizontal)

    # ---- First Segment (bending downward) ----
    p0 = np.array([0.0, 0.0])
    # Center for the downward bend is to the right of the tangent
    c1 = p0 + r1 * np.array([np.sin(theta0), -np.cos(theta0)])
    # Compute the angle from the center to p0
    alpha0 = np.arctan2(p0[1] - c1[1], p0[0] - c1[0])
    dtheta1 = -l1 / r1  # Negative angular span for downward bend
    num1 = num_points // 2
    alpha_vals1 = np.linspace(alpha0, alpha0 + dtheta1, num1)
    arc1_x = c1[0] + r1 * np.cos(alpha_vals1)
    arc1_y = c1[1] + r1 * np.sin(alpha_vals1)

    # ---- Second Segment (continuing downward) ----
    p1 = np.array([arc1_x[-1], arc1_y[-1]])
    theta1 = theta0 + dtheta1  # New tangent direction at p1
    # Center for the second arc, offset to the right of the tangent
    c2 = p1 + r2 * np.array([np.sin(theta1), -np.cos(theta1)])
    alpha_start = np.arctan2(p1[1] - c2[1], p1[0] - c2[0])
    dtheta2 = -l2 / r2  # Negative angular span for downward bend
    num2 = num_points - num1
    alpha_vals2 = np.linspace(alpha_start, alpha_start + dtheta2, num2)
    arc2_x = c2[0] + r2 * np.cos(alpha_vals2)
    arc2_y = c2[1] + r2 * np.sin(alpha_vals2)

    # Combine the two arcs.
    x = np.concatenate([arc1_x, arc2_x])
    y = np.concatenate([arc1_y, arc2_y])
    return x, y


def plot_legs(radii_pairs, l1, l2):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    # Generate a colormap for different beams.
    colors = plt.cm.viridis(np.linspace(0, 1, len(radii_pairs)))
    
    for i, radii in enumerate(radii_pairs):
        x, y = compute_beam_points(radii, l1, l2, num_points=400)
        ax.plot(x, y, lw=2, color=colors[i],
                label=f"r1 = {radii[0]}, r2 = {radii[1]}")
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    #ax.legend()
    plt.show()

# Example usage:
if __name__ == '__main__':
    # # List of pairs of radii for the two segments.
    # radii_pairs = [(1.5, 1.0), (1.2, 0.9), (1.8, 0.7)]
    # l1 = 1.0  # Arc length of the first segment.
    # l2 = 0.8  # Arc length of the second segment.
    # plot_beams(radii_pairs, l1, l2)
    x_swing,y_swing,x_stance,y_stance = plot_foot_path(100,20,5)
    q,t = calculate_radii_front(x_swing,y_swing,x_stance,y_stance)
    plot_legs(q,106,106)
    print(t)
    

