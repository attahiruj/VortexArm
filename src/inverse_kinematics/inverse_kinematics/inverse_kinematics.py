import math
import numpy as np


def get_point(x, y, z, base, arm1, arm2, alpha_rad):
    """Calculate intermediate joint position given target coordinates and angle."""
    e = (x**2 + y**2 + z**2 - base**2 + arm1**2 - arm2**2)/2

    if x != 0 or y != 0:
        # Calculate intermediate point position
        x_y_term = x + y * math.tan(alpha_rad)
        if x_y_term == 0:
            # Handle division by zero case
            a, b = 0, 0
            c = base + e / (z - base) if z != base else base + arm1
        else:
            z_base = z - base
            tan_term = 1 + math.tan(alpha_rad)**2

            a1 = ((z_base**2 * tan_term) / (x_y_term**2)) + 1
            b1 = 2 * ((e * z_base * tan_term) / (x_y_term**2) + base)
            c1 = ((e**2 * tan_term) / (x_y_term**2)) + base**2 - arm1**2

            discriminant = b1**2 - 4 * a1 * c1

            if discriminant < 0:
                # No solution exists, use approximation
                c = base + arm1 * (z - base) / math.sqrt(x**2 + y**2 + (z-base)**2)
            else:
                # Use the positive solution for elbow-up configuration
                d1 = math.sqrt(discriminant)
                c = (b1 + d1) / (2 * a1)

            a = (e - c * z_base) / x_y_term
            b = a * math.tan(alpha_rad)
    else:
        # Handle special case when x and y are both 0
        a, b = 0, 0
        c = min(base + arm1, z)  # Ensure c doesn't exceed z

    return a, b, c

def get_angles(x, y, z, base, arm1, arm2):
    """Calculate joint angles for the robotic arm."""
    # Calculate distance from base joint to target
    d = math.sqrt(x**2 + y**2 + (z-base)**2)

    # Calculate base rotation angle (around z-axis)
    if x != 0:
        alpha = math.atan2(y, x)  # Using atan2 handles all quadrants correctly
    else:
        alpha = math.pi/2 if y >= 0 else -math.pi/2

    # Calculate shoulder angle (between base and arm1)
    if d > arm1 + arm2 - 0.001:  # Allow tiny rounding errors
        # Target is at maximum reach, arm fully extended
        theta1 = math.atan2(z-base, math.sqrt(x**2 + y**2))
        theta2 = 0
    else:
        # Law of cosines for shoulder angle
        theta1 = math.asin((z-base)/d) if d != 0 else 0
        cos_theta2 = (arm1**2 + d**2 - arm2**2)/(2*arm1*d)
        # Clamp to valid range to avoid numerical errors
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2 = math.acos(cos_theta2)

    beta = theta1 + theta2  # Shoulder angle

    # Calculate elbow angle (between arm1 and arm2)
    cos_gamma = (arm1**2 + arm2**2 - d**2)/(2*arm1*arm2)
    # Clamp to valid range to avoid numerical errors
    cos_gamma = max(min(cos_gamma, 1.0), -1.0)
    gamma = math.acos(cos_gamma)

    # Convert to degrees
    alpha_deg = math.degrees(alpha)
    beta_deg = math.degrees(beta)
    gamma_deg = math.degrees(gamma)

    return alpha_deg, beta_deg, gamma_deg