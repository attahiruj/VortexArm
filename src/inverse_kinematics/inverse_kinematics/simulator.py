"""
3D Robotic Arm Simulation with Inverse Kinematics
This script simulates a 3D robotic arm using inverse kinematics.

Author: Attahiru Jibril
Date: 2025-04-14
"""

import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation
from inverse_kinematics import get_angles, get_point
import matplotlib


plt.ion()                   # Enable interactive mode for matplotlib
matplotlib.use('TkAgg')     # Use TkAgg backend for interactive plotting


def run_simulation(robot_params):
    base = robot_params["base"]
    arm1 = robot_params["arm1"]
    arm2 = robot_params["arm2"]

    max_reach = arm1 + arm2

    # Set up the figure and 3D axis
    fig = plt.figure(figsize=(10, 8))
    plt.get_current_fig_manager().window.wm_geometry("+50+50")
    ax = fig.add_subplot(111, projection='3d')

    plt.subplots_adjust(bottom=0.25)

    ax_x = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_y = plt.axes([0.25, 0.10, 0.65, 0.03])
    ax_z = plt.axes([0.25, 0.05, 0.65, 0.03])

    # Create sliders for X, Y, Z coordinates
    x_slider = Slider(ax_x, 'X', -max_reach, max_reach, valinit=150)
    y_slider = Slider(ax_y, 'Y', -max_reach, max_reach, valinit=0)
    z_slider = Slider(ax_z, 'Z', 0, max_reach + base, valinit=150)

    def update_plot(frame):
        """
        Updates the 3D plot of the robotic arm based on the current slider values and target position.

        Behavior:
            - Retrieves the target coordinates (x, y, z) from the sliders.
            - Calculates the current reach of the robotic arm and checks if the target is within the maximum reach.
            - If the target is reachable:
                - Computes the joint angles (alpha, beta, gamma) and intermediate joint positions.
                - Updates the 3D plot with the arm's segments and joint positions.
                - Displays the joint angles and target coordinates as text annotations on the plot.
            - If the target is out of reach:
                - Displays a warning message on the plot.
            - Redraws the canvas to reflect the updates.

        Parameters:
            frame (int): The current frame index (required by animation functions but not used here).
        """

        # Get the current slider values
        x, y, z = x_slider.val, y_slider.val, z_slider.val
        current_reach = math.sqrt(x**2 + y**2 + (z - base)**2)
        ax.clear()

        if current_reach <= max_reach:
            alpha_deg, beta_deg, gamma_deg = get_angles(x, y, z, robot_params)
            alpha_rad = math.radians(alpha_deg)
            joint2_x, joint2_y, joint2_z = get_point(x, y, z, alpha_rad, robot_params)

            # Plot the robotic arm
            ax.plot([0, 0], [0, 0], [0, base], 'k-', linewidth=3)
            ax.plot([0, joint2_x], [0, joint2_y], [base, joint2_z], 'b-', linewidth=3)
            ax.plot([joint2_x, x], [joint2_y, y], [joint2_z, z], 'g-', linewidth=3)

            ax.scatter([0], [0], [0], color='r', s=100)
            ax.scatter([0], [0], [base], color='r', s=100)
            ax.scatter([joint2_x], [joint2_y], [joint2_z], color='r', s=100)
            ax.scatter([x], [y], [z], color='r', s=100)

            # Display angles and coordinates
            status_text = f"Base angle: {alpha_deg:.1f}°\nShoulder angle: {beta_deg:.1f}°\nElbow angle: {gamma_deg:.1f}°"
            ax.text2D(0.02, 0.95,
                      status_text,
                      transform=ax.transAxes,
                      fontsize=10,
                      verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
                    )

            coord_text = f"Target: ({x:.1f}, {y:.1f}, {z:.1f})"
            ax.text2D(0.02, 0.80,
                      coord_text,
                      transform=ax.transAxes,
                      fontsize=10,
                      verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5)
                    )
        else:
            ax.text2D(0.5, 0.5,
                      "Target out of reach!",
                      transform=ax.transAxes,
                      fontsize=14,
                      color='red', ha='center', va='center',
                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
                    )

        max_dim = max(base, arm1, arm2) * 1.5

        ax.set_xlim(-max_dim, max_dim)
        ax.set_ylim(-max_dim, max_dim)
        ax.set_zlim(0, max_dim * 1.2)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        ax.set_title('3D Robotic Arm Visualization')

        fig.canvas.draw_idle()

    # Animation setup
    ani = animation.FuncAnimation(fig, update_plot, frames=1, interval=50, blit=False)

    # Slider update function
    def update(_):
        ani._start()

    # Connect sliders to update function
    x_slider.on_changed(update)
    y_slider.on_changed(update)
    z_slider.on_changed(update)

    # Reset button
    reset_ax = plt.axes([0.8, 0.0, 0.1, 0.04])
    reset_button = Button(reset_ax, 'Reset')

    # Define reset function
    def reset(_):
        x_slider.reset()
        y_slider.reset()
        z_slider.reset()
        update(_)

    # Connect reset button to reset function
    reset_button.on_clicked(reset)

    # View buttons
    # Define view buttons
    front_view_ax = plt.axes([0.05, 0.0, 0.1, 0.04])
    left_view_ax = plt.axes([0.16, 0.0, 0.1, 0.04])
    top_view_ax = plt.axes([0.27, 0.0, 0.1, 0.04])
    corner_view_ax = plt.axes([0.38, 0.0, 0.1, 0.04])

    front_view_button = Button(front_view_ax, 'Front')
    left_view_button = Button(left_view_ax, 'Left')
    top_view_button = Button(top_view_ax, 'Top')
    corner_view_button = Button(corner_view_ax, '3D')

    # Define view functions
    def set_front_view(_):
        # Front view (looking at YZ plane)
        ax.view_init(elev=0, azim=0)
        fig.canvas.draw()

    def set_left_view(_):
        # Left side view (looking at XZ plane from negative Y direction)
        ax.view_init(elev=0, azim=-90)
        fig.canvas.draw()

    def set_top_view(_):
        # Top view (looking at XY plane)
        ax.view_init(elev=90, azim=90)
        fig.canvas.draw()

    def set_corner_view(_):
        # 3D corner view (looking from above at a corner)
        ax.view_init(elev=35, azim=45)
        fig.canvas.draw()

    # Connect view buttons to their respective functions
    front_view_button.on_clicked(set_front_view)
    left_view_button.on_clicked(set_left_view)
    top_view_button.on_clicked(set_top_view)
    corner_view_button.on_clicked(set_corner_view)

    # Set initial view
    plt.get_current_fig_manager().set_window_title('3D Robotic Arm IK Visualization')
    update_plot(0)
    plt.show(block=True)
