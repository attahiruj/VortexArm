"""
Inverse Kinematics Operator
This operator computes the joint angles required to position a robotic arm
at a specified 3D coordinate.

It uses inverse kinematics to determine the angles based on the arm's geometry
and the target position.

Author: Attahiru Jibril (Modified)
Date: 2025-04-20
"""

import numpy as np
import pyarrow as pa
import math

from dora import DoraStatus, Node
from robot_config import robot_params, servo_config


class Operator:
    """
    Inverse Kinematics Operator
    
    Takes 3D position coordinates as input and outputs the required joint angles
    to position a robotic arm at that location.
    """

    def __init__(self):
        self.params = robot_params
        self.servo_config = servo_config

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INIT":
            # Initialize operator - nothing special needed here
            pass
        elif dora_event["type"] == "INPUT":
            # Check if this is a position input
            if dora_event["id"] == "position":
                # Extract position coordinates from input
                position = dora_event["value"].to_numpy()
                
                # Ensure we have 3 coordinates (x, y, z)
                if len(position) >= 3:
                    x, y, z = position[:3]
                    
                    # Calculate joint angles using inverse kinematics
                    alpha, beta, gamma = self.get_angles(x, y, z, self.params)
                    
                    # Apply servo limits to the calculated angles
                    alpha = self.apply_angle_limits(alpha, "base")
                    beta = self.apply_angle_limits(beta, "shoulder")
                    gamma = self.apply_angle_limits(gamma, "elbow")
                    
                    # Send joint angles as output
                    send_output("joints", pa.array([alpha, beta, gamma]), dora_event["metadata"])
            elif dora_event["id"] == "tick":
                # Received a tick event - we could send default joint values here if needed
                pass
                
        return DoraStatus.CONTINUE
    
    def apply_angle_limits(self, angle, servo_name):
        """Apply servo angle limits to the calculated angle."""
        if servo_name in self.servo_config:
            min_angle = self.servo_config[servo_name]["min_angle"]
            max_angle = self.servo_config[servo_name]["max_angle"]
            
            if angle < min_angle or angle > max_angle:
                limited_angle = max(min(angle, max_angle), min_angle)
                print(f"Limiting {servo_name} angle from {angle:.1f}° to {limited_angle:.1f}° (range: {min_angle}° to {max_angle}°)")
                return limited_angle
        
        return angle
    
    def get_angles(self, x: float, y: float, z: float, params: dict) -> tuple:
        """
        Compute the joint angles of a 3DOF robotic arm to reach a specific
        target point (x, y, z).

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

        # Check if the target is within reach
        max_reach = arm1 + arm2
        if d > max_reach - 0.001:
            print(f"Target ({x}, {y}, {z}) is beyond arm's reach. Distance: {d}, Max reach: {max_reach}")
            # Scale the target to be at max reach
            scale_factor = max_reach / d
            # Keep same direction but scale to max reach
            x_scaled = x * scale_factor
            y_scaled = y * scale_factor
            z_scaled = (z - base) * scale_factor + base
            print(f"Scaling target to ({x_scaled}, {y_scaled}, {z_scaled})")
            # Recalculate distance with scaled target
            d = max_reach - 0.001
            
            # Optionally update x, y, z if you want the calculation to use the scaled values
            # x, y, z = x_scaled, y_scaled, z_scaled
        
        # Calculate elevation angle
        elevation = math.atan2(z - base, math.sqrt(x**2 + y**2))
        
        # Law of cosines to compute angle between arm segments
        # cos_gamma represents the angle between the two arm segments
        cos_gamma = (arm1**2 + arm2**2 - d**2) / (2 * arm1 * arm2)
        cos_gamma = max(min(cos_gamma, 1.0), -1.0)  # Clamp to valid range
        gamma = math.acos(cos_gamma)  # Internal angle between segments
        
        # Law of cosines for the shoulder angle
        cos_beta = (arm1**2 + d**2 - arm2**2) / (2 * arm1 * d)
        cos_beta = max(min(cos_beta, 1.0), -1.0)  # Clamp to valid range
        shoulder_to_target = math.acos(cos_beta)
        
        # Final shoulder angle calculation
        beta = elevation + shoulder_to_target
        
        # Convert to degrees
        alpha_deg = math.degrees(alpha)
        beta_deg = math.degrees(beta)
        gamma_deg = 180 - math.degrees(gamma)  # Convert to external angle for the elbow
        
        # Apply coordinate system conversions to match real hardware
        # The sign of these adjustments may need to be inverted depending on your servo arrangement
        
        # Adjust shoulder angle to match the servo configuration
        # For shoulder: 0° should be horizontal, positive up (based on your setup)
        beta_deg = 90 - beta_deg  # This makes 0° horizontal, positive up
        
        # Adjust elbow angle to match the servo configuration
        # The elbow angle is relative to the shoulder arm segment
        # A positive value should bend the elbow upward
        
        print(f"Calculated raw angles - Alpha: {alpha_deg:.1f}°, Beta: {beta_deg:.1f}°, Gamma: {gamma_deg:.1f}°")
        
        return alpha_deg, beta_deg, gamma_deg


def main():
    """
    Main entry point for the inverse kinematics operator.
    This function creates a Dora node and connects the operator to it.
    """
    node = Node()
    operator = Operator()
    
    # Process events in a loop to maintain the connection
    for event in node:
        status = operator.on_event(
            event,
            lambda output_id, data, metadata: node.send_output(output_id, data, metadata)
        )
        
        if status == DoraStatus.STOP:
            break
    
    return 0


if __name__ == "__main__":
    main()