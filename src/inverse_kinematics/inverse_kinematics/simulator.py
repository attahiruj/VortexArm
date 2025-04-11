import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation

from inverse_kinematics import get_point
from inverse_kinematics import get_angles


def update_plot(fig, ax, x_slider, y_slider, z_slider, robot_params):
    """Update function for animation."""
    x = x_slider.val
    y = y_slider.val
    z = z_slider.val

    base, arm1, arm2 = robot_params
    max_reach = arm1 + arm2
    current_reach = math.sqrt(x**2 + y**2 + (z-base)**2)

    ax.clear()

    if current_reach <= max_reach:
        # Calculate joint angles and positions
        alpha_deg, beta_deg, gamma_deg = get_angles(x, y, z, base, arm1, arm2)
        alpha_rad = math.radians(alpha_deg)
        joint2_x, joint2_y, joint2_z = get_point(x, y, z,
                                                 base, arm1, arm2, alpha_rad)

        # Plot the arm segments
        # Base to first joint
        ax.plot([0, 0], [0, 0], [0, base], 'k-', linewidth=3)

        # First joint to second joint
        ax.plot([0, joint2_x], [0, joint2_y], [base, joint2_z], 'b-', linewidth=3)

        # Second joint to end effector
        ax.plot([joint2_x, x], [joint2_y, y], [joint2_z, z], 'g-', linewidth=3)

        # Plot the joints
        ax.scatter([0], [0], [0], color='r', s=100)  # Base origin
        ax.scatter([0], [0], [base], color='r', s=100)  # First joint
        ax.scatter([joint2_x], [joint2_y], [joint2_z], color='r', s=100)  # Second joint
        ax.scatter([x], [y], [z], color='r', s=100)  # End effector

        # Display angles
        status_text = f"Base angle: {alpha_deg:.1f}°\nShoulder angle: {beta_deg:.1f}°\nElbow angle: {gamma_deg:.1f}°"
        ax.text2D(0.02, 0.95, status_text, transform=ax.transAxes, fontsize=10,
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Display coordinates
        coord_text = f"Target: ({x:.1f}, {y:.1f}, {z:.1f})"
        ax.text2D(0.02, 0.80, coord_text, transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    else:
        # Display out of reach message
        ax.text2D(0.5, 0.5, "Target out of reach!", transform=ax.transAxes, fontsize=14,
                 color='red', ha='center', va='center', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # Set plot limits and labels
    max_dim = max(base, arm1, arm2) * 1.5
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(0, max_dim * 1.2)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('3D Robotic Arm Visualization')

    # Update the figure
    fig.canvas.draw_idle()


def setup_visualization(robot_params):
    """Set up the visualization environment."""
    base, arm1, arm2 = robot_params
    max_reach = arm1 + arm2

    # Enable interactive mode and use TkAgg backend for better performance
    plt.ion()
    matplotlib.use('TkAgg')

    # Create the figure and 3D axis
    fig = plt.figure(figsize=(10, 8))
    plt.get_current_fig_manager().window.wm_geometry("+50+50")  # Position window
    ax = fig.add_subplot(111, projection='3d')

    # Adjust the main plot area
    plt.subplots_adjust(bottom=0.25)

    # Create sliders for x, y, z coordinates
    ax_x = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_y = plt.axes([0.25, 0.10, 0.65, 0.03])
    ax_z = plt.axes([0.25, 0.05, 0.65, 0.03])

    # Create the sliders
    x_slider = Slider(ax_x, 'X', -max_reach, max_reach, valinit=150)
    y_slider = Slider(ax_y, 'Y', -max_reach, max_reach, valinit=0)
    z_slider = Slider(ax_z, 'Z', 0, max_reach + base, valinit=150)

    return fig, ax, x_slider, y_slider, z_slider


def setup_controls(fig, ax, x_slider, y_slider, z_slider, ani, robot_params):
    """Set up control buttons for the visualization."""
    # Update plot when sliders change
    def update(_):
        ani._start()

    x_slider.on_changed(update)
    y_slider.on_changed(update)
    z_slider.on_changed(update)

    # Set an initial view angle
    ax.view_init(elev=30, azim=45)

    # Add a reset button
    reset_ax = plt.axes([0.8, 0.0, 0.1, 0.04])
    reset_button = Button(reset_ax, 'Reset')

    def reset(_):
        x_slider.reset()
        y_slider.reset()
        z_slider.reset()
        update(_)

    reset_button.on_clicked(reset)

    # Add view buttons
    front_view_ax = plt.axes([0.05, 0.0, 0.1, 0.04])
    left_view_ax = plt.axes([0.16, 0.0, 0.1, 0.04])
    top_view_ax = plt.axes([0.27, 0.0, 0.1, 0.04])
    corner_view_ax = plt.axes([0.38, 0.0, 0.1, 0.04])

    front_view_button = Button(front_view_ax, 'Front')
    left_view_button = Button(left_view_ax, 'Left')
    top_view_button = Button(top_view_ax, 'Top')
    corner_view_button = Button(corner_view_ax, '3D')

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

    front_view_button.on_clicked(set_front_view)
    left_view_button.on_clicked(set_left_view)
    top_view_button.on_clicked(set_top_view)
    corner_view_button.on_clicked(set_corner_view)

    # Set the window title
    plt.get_current_fig_manager().set_window_title('3D Robotic Arm Visualization')


def run_simulator():
    """Run the 3D robotic arm simulator."""
    
    # Robot dimensions
    robot_params = (100, 204, 165)  # base, arm1, arm2

    # Setup visualization
    fig, ax, x_slider, y_slider, z_slider = setup_visualization(robot_params)

    # Create animation
    ani = animation.FuncAnimation(
        fig, update_plot,
        fargs=(x_slider, y_slider, z_slider, robot_params),
        frames=1, interval=50, blit=False)

    # Setup controls
    setup_controls(fig, ax, x_slider, y_slider, z_slider, ani, robot_params)

    # Initial update
    update_plot(fig, ax, x_slider, y_slider, z_slider, robot_params)

    # Use block=True to ensure window stays open
    plt.show(block=True)