"""
Inverse Kinematics for a 3DOF Robotic Arm

This module provides functions to compute the joint positions and angles of a
3 Degrees of Freedom (3DOF) robotic arm given a target coordinate in 3D space.
It is useful for robotic motion planning and control.

Author: Attahiru Jibril
Date: 2025-04-14
"""

import math


def get_point(x: float, y: float, z: float,
              alpha_rad: float, params: dict) -> tuple:
    """
    Calculate the joint position (a, b, c) of a 3DOF robotic arm
    for a given target point in 3D space.

    This function solves the inverse kinematics problem using geometric
    relationships. It determines where the intermediate joint of the
    robotic arm should be to reach the desired end-effector position.

    Parameters:
        x (float): Target x-coordinate of the end effector.
        y (float): Target y-coordinate of the end effector.
        z (float): Target z-coordinate of the end effector.
        alpha_rad (float): Base rotation angle (alpha) in radians.
        params (dict): Dictionary containing robotic arm parameters:
            - base (float): Height of the base from the ground.
            - arm1 (float): Length of the first arm segment.
            - arm2 (float): Length of the second arm segment.

    Returns:
        tuple: Coordinates (a, b, c) representing the position of the
               intermediate joint between arm1 and arm2.
    """

    base, arm1, arm2 = params["base"], params["arm1"], params["arm2"]

    # Using the cosine law projection to find a helper variable `e`
    e = (x**2 + y**2 + z**2 - base**2 + arm1**2 - arm2**2) / 2

    # General case where x and y are not both zero
    if x != 0 or y != 0:
        x_y_term = x + y * math.tan(alpha_rad)

        # Prevent division by zero if x_y_term is 0
        if x_y_term == 0:
            a, b = 0, 0
            c = base + e / (z - base) if z != base else base + arm1
        else:
            z_base = z - base
            tan_term = 1 + math.tan(alpha_rad)**2

            # Coefficients for solving the quadratic equation to find `c`
            a1 = ((z_base**2 * tan_term) / (x_y_term**2)) + 1
            b1 = 2 * ((e * z_base * tan_term) / (x_y_term**2) + base)
            c1 = ((e**2 * tan_term) / (x_y_term**2)) + base**2 - arm1**2

            discriminant = b1**2 - 4 * a1 * c1

            # Check if the solution is real
            if discriminant < 0:
                # Fallback to a projection method if no real solution
                c = base + arm1 * (z - base) / math.sqrt(x**2 + y**2 + (z - base)**2)
            else:
                d1 = math.sqrt(discriminant)
                c = (b1 + d1) / (2 * a1)

            # Back-substitute to find `a` and `b` coordinates
            a = (e - c * z_base) / x_y_term
            b = a * math.tan(alpha_rad)
    else:
        # Special case: target is directly above or below base
        a, b = 0, 0
        c = min(base + arm1, z)

    return a, b, c


def get_angles(x: float, y: float, z: float, params: dict) -> tuple:
    """
    Compute the joint angles of a 3DOF robotic arm to reach a specific
    target point (x, y, z).

    This function uses inverse kinematics and trigonometry to find the
    required angles for each joint:
        - alpha: Base rotation angle (in degrees)
        - beta: Shoulder angle relative to the base (in degrees)
        - gamma: Elbow angle between arm1 and arm2 (in degrees)

    Parameters:
        x (float): Target x-coordinate of the end effector.
        y (float): Target y-coordinate of the end effector.
        z (float): Target z-coordinate of the end effector.
        params (dict): Dictionary containing robotic arm parameters:
            - base (float): Height of the base from the ground.
            - arm1 (float): Length of the first arm segment.
            - arm2 (float): Length of the second arm segment.

    Returns:
        tuple: Angles (alpha, beta, gamma) in degrees.
    """
    base, arm1, arm2 = params["base"], params["arm1"], params["arm2"]

    # Compute straight-line distance from the base joint to target point
    d = math.sqrt(x**2 + y**2 + (z - base)**2)

    # Compute base rotation angle alpha (yaw)
    if x != 0:
        alpha = math.atan2(y, x)
    else:
        alpha = math.pi / 2 if y >= 0 else -math.pi / 2

    # Check if the target is out of reach
    if d > arm1 + arm2 - 0.001:
        # Fully extend the arm in direction of the target
        theta1 = math.atan2(z - base, math.sqrt(x**2 + y**2))
        theta2 = 0  # No elbow bend
    else:
        # Calculate shoulder angle
        theta1 = math.asin((z - base) / d) if d != 0 else 0

        # Law of cosines to compute angle between arm1 and the vector to target
        cos_theta2 = (arm1**2 + d**2 - arm2**2) / (2 * arm1 * d)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)  # Clamp to valid range
        theta2 = math.acos(cos_theta2)

    beta = theta1 + theta2  # Shoulder joint angle relative to base

    # Elbow angle using cosine law again
    cos_gamma = (arm1**2 + arm2**2 - d**2) / (2 * arm1 * arm2)
    cos_gamma = max(min(cos_gamma, 1.0), -1.0)
    gamma = math.acos(cos_gamma)

    return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)
