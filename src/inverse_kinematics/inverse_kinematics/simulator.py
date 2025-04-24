"""
3D Robotic Arm Simulation with Inverse Kinematics - Dora Integration
This script provides a visualization of a 3D robotic arm and publishes position
data to the Dora system directly when the sliders are moved.

Author: Attahiru Jibril (Modified)
Date: 2025-04-20
"""

import math
import numpy as np
import pyarrow as pa
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.animation as animation
import matplotlib
from dora import DoraStatus
from robot_config import servo_config


plt.ion()                   # Enable interactive mode for matplotlib
matplotlib.use('TkAgg')     # Use TkAgg backend for interactive plotting


class Operator:
    """
    3D Robotic Arm Simulator Operator

    Provides a visualization of the robotic arm and automatically publishes
    target position coordinates to the Dora system when sliders are moved.
    """

    def __init__(self):
        """Initialize the simulator operator."""
        self.fig = None
        self.ax = None
        self.animation = None
        self.target_position = [150, 0, 150]  # Default target position
        self.send_output_fn = None
        self.run_state = True
        self.robot_params = None
        self.servo_config = servo_config

    def get_angles(self, x: float, y: float, z: float, params: dict) -> tuple:
        """
        Compute the joint angles of a 3DOF robotic arm to reach a specific
        target point (x, y, z).

        This function calculates angles corresponding to the inverse kinematics solution,
        and applies the proper sign conventions to match the physical robot.
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
            print(f"Target out of reach. Distance: {d}, Max reach: {arm1+arm2}")
            # Scale the target to be at max reach
            scale_factor = (arm1 + arm2 - 0.001) / d
            # Keep same direction but scale to max reach
            x = x * scale_factor
            y = y * scale_factor
            z = base + (z - base) * scale_factor
            # Recalculate distance with scaled target
            d = math.sqrt(x**2 + y**2 + (z - base)**2)
            print(f"Scaled target to ({x:.1f}, {y:.1f}, {z:.1f})")

        # Calculate elevation angle from horizontal plane
        elevation = math.atan2(z - base, math.sqrt(x**2 + y**2))

        # Law of cosines for elbow angle (between the two arm segments)
        cos_gamma = (arm1**2 + arm2**2 - d**2) / (2 * arm1 * arm2)
        cos_gamma = max(min(cos_gamma, 1.0), -1.0)  # Clamp to valid range
        gamma = math.acos(cos_gamma)  # Internal angle between segments

        # Law of cosines for shoulder angle
        cos_beta = (arm1**2 + d**2 - arm2**2) / (2 * arm1 * d)
        cos_beta = max(min(cos_beta, 1.0), -1.0)  # Clamp to valid range
        shoulder_to_target = math.acos(cos_beta)

        # Final shoulder angle calculation
        beta = elevation + shoulder_to_target

        # Convert to degrees and apply sign conventions to match physical robot
        alpha_deg = math.degrees(alpha)
        beta_deg = math.degrees(beta)
        gamma_deg = 180 - math.degrees(gamma)  # Convert to external angle

        # Apply coordinate system adjustments to match real hardware
        # These adjustments depend on how your physical robot is configured
        # For the simulation to match reality:

        # For shoulder: Adjust so 0° is horizontal
        beta_deg = 90 - beta_deg  # This makes 0° horizontal, positive up

        # Apply servo limits
        alpha_deg = self.apply_angle_limits(alpha_deg, "base")
        beta_deg = self.apply_angle_limits(beta_deg, "shoulder")
        gamma_deg = self.apply_angle_limits(gamma_deg, "elbow")

        return alpha_deg, beta_deg, gamma_deg

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

    def get_point(self, x: float, y: float, z: float,
              alpha_rad: float, beta_rad: float, gamma_rad: float, params: dict) -> tuple:
        """
        Calculate the joint positions for a 3DOF robotic arm based on the calculated angles.

        This is used to display the arm in the simulator visualization.

        Args:
            x, y, z: Target end effector coordinates
            alpha_rad: Base rotation angle in radians
            beta_rad: Shoulder angle in radians (adjusted for simulator)
            gamma_rad: Elbow angle in radians (adjusted for simulator)
            params: Robot parameters

        Returns:
            tuple: (joint2_x, joint2_y, joint2_z, end_x, end_y, end_z)
                The coordinates of the shoulder-elbow joint and end effector
        """
        base, arm1, arm2 = params["base"], params["arm1"], params["arm2"]

        # Convert beta back from adjusted angle to elevation angle
        # If beta_deg = 90 - elevation_deg, then elevation_rad = pi/2 - beta_rad
        elevation_rad = math.pi/2 - beta_rad

        # Calculate joint positions in 3D space
        # First, determine the projection length on the XY plane for arm1
        proj_length_arm1 = arm1 * math.cos(elevation_rad)

        # Calculate the X and Y coordinates of the elbow joint
        joint2_x = proj_length_arm1 * math.cos(alpha_rad)
        joint2_y = proj_length_arm1 * math.sin(alpha_rad)

        # Calculate the Z coordinate of the elbow joint
        joint2_z = base + arm1 * math.sin(elevation_rad)

        # Convert gamma to internal angle for calculation
        internal_gamma_rad = math.pi - gamma_rad

        # Calculate direction vector for arm2 from elbow joint
        # First get angle of arm2 relative to horizontal plane
        arm2_elevation = elevation_rad + internal_gamma_rad - math.pi/2

        # Calculate the projection length on the XY plane for arm2
        proj_length_arm2 = arm2 * math.cos(arm2_elevation)

        # Calculate end effector coordinates
        end_x = joint2_x + proj_length_arm2 * math.cos(alpha_rad)
        end_y = joint2_y + proj_length_arm2 * math.sin(alpha_rad)
        end_z = joint2_z + arm2 * math.sin(arm2_elevation)

        return joint2_x, joint2_y, joint2_z, end_x, end_y, end_z

    def run_simulation(self, robot_params, send_output):
        """
        Create and run the 3D simulation with sliders.
        """
        self.robot_params = robot_params
        self.send_output_fn = send_output

        base = robot_params["base"]
        arm1 = robot_params["arm1"]
        arm2 = robot_params["arm2"]

        max_reach = arm1 + arm2

        # Set up the figure and 3D axis
        self.fig = plt.figure(figsize=(10, 8))
        plt.get_current_fig_manager().window.wm_geometry("+50+50")
        self.ax = self.fig.add_subplot(111, projection='3d')

        plt.subplots_adjust(bottom=0.25)

        ax_x = plt.axes([0.25, 0.15, 0.65, 0.03])
        ax_y = plt.axes([0.25, 0.10, 0.65, 0.03])
        ax_z = plt.axes([0.25, 0.05, 0.65, 0.03])

        # Create sliders for X, Y, Z coordinates
        x_slider = Slider(ax_x, 'X', -max_reach, max_reach, valinit=self.target_position[0])
        y_slider = Slider(ax_y, 'Y', -max_reach, max_reach, valinit=self.target_position[1])
        z_slider = Slider(ax_z, 'Z', 0, max_reach + base, valinit=self.target_position[2])

        def update_plot(frame):
            """Update the 3D plot of the robotic arm based on slider values."""
            # Get the current slider values
            x, y, z = x_slider.val, y_slider.val, z_slider.val
            current_reach = math.sqrt(x**2 + y**2 + (z - base)**2)
            self.ax.clear()

            # Update target position
            self.target_position = [x, y, z]

            # Publish position to Dora system
            if self.send_output_fn:
                self.send_output_fn("position", pa.array(self.target_position), {})

            # Calculate joint angles using inverse kinematics
            alpha_deg, beta_deg, gamma_deg = self.get_angles(x, y, z, robot_params)

            # Convert angles to radians for visualization
            alpha_rad = math.radians(alpha_deg)
            beta_rad = math.radians(beta_deg)
            gamma_rad = math.radians(gamma_deg)

            # Calculate joint positions for visualization
            joint2_x, joint2_y, joint2_z, end_x, end_y, end_z = self.get_point(
                x, y, z, alpha_rad, beta_rad, gamma_rad, robot_params
            )

            # If target is too far, use calculated end position instead of target
            if current_reach > max_reach:
                x, y, z = end_x, end_y, end_z

            # Plot the robotic arm
            self.ax.plot([0, 0], [0, 0], [0, base], 'k-', linewidth=3)  # Base
            self.ax.plot([0, joint2_x], [0, joint2_y], [base, joint2_z], 'b-', linewidth=3)  # Arm1
            self.ax.plot([joint2_x, end_x], [joint2_y, end_y], [joint2_z, end_z], 'g-', linewidth=3)  # Arm2

            # Plot the joints and end effector
            self.ax.scatter([0], [0], [0], color='k', s=50)  # Ground origin
            self.ax.scatter([0], [0], [base], color='r', s=100)  # Base joint
            self.ax.scatter([joint2_x], [joint2_y], [joint2_z], color='r', s=100)  # Elbow joint
            self.ax.scatter([end_x], [end_y], [end_z], color='r', s=100)  # End effector

            # Draw target indicator if different from end effector (out of reach case)
            if current_reach > max_reach:
                self.ax.scatter([x_slider.val], [y_slider.val], [z_slider.val],
                                color='orange', s=50, alpha=0.5, marker='x')  # Target

            # Add servo limit info to status text
            base_limits = f"Base: {self.servo_config['base']['min_angle']}° to {self.servo_config['base']['max_angle']}°"
            shoulder_limits = f"Shoulder: {self.servo_config['shoulder']['min_angle']}° to {self.servo_config['shoulder']['max_angle']}°"
            elbow_limits = f"Elbow: {self.servo_config['elbow']['min_angle']}° to {self.servo_config['elbow']['max_angle']}°"

            # Display angles and coordinates
            if current_reach <= max_reach:
                target_text = f"Target: ({x:.1f}, {y:.1f}, {z:.1f})"
            else:
                target_text = f"Target: ({x_slider.val:.1f}, {y_slider.val:.1f}, {z_slider.val:.1f}) - OUT OF REACH\nActual: ({end_x:.1f}, {end_y:.1f}, {end_z:.1f})"

            status_text = (f"Joint Angles:\n"
                           f"Base: {alpha_deg:.1f}°\n"
                           f"Shoulder: {beta_deg:.1f}°\n"
                           f"Elbow: {gamma_deg:.1f}°\n\n"
                           f"Limits:\n{base_limits}\n{shoulder_limits}\n{elbow_limits}")

            self.ax.text2D(0.02, 0.95,
                      status_text,
                      transform=self.ax.transAxes,
                      fontsize=10,
                      verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
                    )

            self.ax.text2D(0.02, 0.60,
                      target_text,
                      transform=self.ax.transAxes,
                      fontsize=10,
                      verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5)
                    )

            max_dim = max(base, arm1, arm2) * 1.5

            self.ax.set_xlim(-max_dim, max_dim)
            self.ax.set_ylim(-max_dim, max_dim)
            self.ax.set_zlim(0, max_dim * 1.2)

            self.ax.set_xlabel('X axis')
            self.ax.set_ylabel('Y axis')
            self.ax.set_zlabel('Z axis')

            self.ax.set_title('3D Robotic Arm Visualization (Dora Integration)')

            self.fig.canvas.draw_idle()

            # Return continuation status
            return self.run_state

        # Animation setup
        self.animation = animation.FuncAnimation(self.fig, update_plot, frames=None, interval=50, blit=False)

        # Make sure the figure is shown before adding callbacks
        plt.show(block=False)
        self.fig.canvas.draw_idle()

        # Slider update function
        def update(_):
            self.animation._start()

        # Connect sliders to update function
        x_slider.on_changed(update)
        y_slider.on_changed(update)
        z_slider.on_changed(update)

        # Set initial view and window title
        plt.get_current_fig_manager().set_window_title('3D Robotic Arm IK Visualization (Dora)')
        update_plot(0)

        # Initial position publish
        self.send_output_fn("position", pa.array(self.target_position), {})

        # Non-blocking show
        plt.show(block=False)

    def on_event(self, dora_event, send_output):
        """
        Process Dora events and handle simulator lifecycle.
        """
        if dora_event["type"] == "INIT":
            from robot_config import robot_params
            # Start the simulation in a non-blocking way
            if self.fig is None:  # Only initialize if not already initialized
                print("Initializing simulator with GUI...")
                self.run_simulation(robot_params, send_output)

                # Initial position publish
                send_output("position", pa.array(self.target_position), {})

        elif dora_event["type"] == "INPUT":
            # If we receive a tick, republish the position
            if dora_event["id"] == "tick":
                if self.target_position and self.send_output_fn:
                    self.send_output_fn("position", pa.array(self.target_position), {})

        elif dora_event["type"] == "STOP":
            # Clean up resources
            self.run_state = False
            if self.fig:
                plt.close(self.fig)

        # Ensure the GUI stays responsive
        if self.fig is not None:
            try:
                # Process GUI events
                plt.pause(0.01)
                return DoraStatus.CONTINUE
            except Exception as e:
                print(f"GUI error: {e}")
                return DoraStatus.CONTINUE

        # Always continue unless explicitly stopped
        return DoraStatus.CONTINUE


# Function to run the simulation directly for testing
def run_simulation(robot_params):
    """Standalone function to run the simulation without Dora."""
    simulator = Operator()

    def dummy_send_output(output_id, data, metadata):
        print(f"Publishing to {output_id}: {data.to_pylist()}")

    simulator.run_simulation(robot_params, dummy_send_output)
    plt.show(block=True)  # Blocking call for standalone mode

    return simulator