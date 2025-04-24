"""
Robot configuration parameters for the inverse kinematics module.
This module defines the parameters for the robot's base and arm lengths,
and servo limits with corrected angle ranges to match real hardware behavior.

Author: Attahiru Jibril (Modified)
Date: 2025-04-20
"""

robot_params = {
    "base": 100,   # Height of the base
    "arm1": 204,   # Length of arm1 (shoulder to elbow)
    "arm2": 165    # Length of arm2 (elbow to wrist)
}

# Servo configuration with expanded angle ranges for shoulder and elbow
# to allow a wider range of motion
servo_config = {
    "base": {
        "pin": 11,       # Servo pin number
        "rest": 1500,    # Rest position (PWM value)
        "min": 600,      # Minimum PWM value
        "max": 2400,     # Maximum PWM value
        "min_angle": -90, # Minimum angle (degrees)
        "max_angle": 90,  # Maximum angle (degrees)
    },
    "shoulder": {
        "pin": 10,
        "rest": 1650,
        "min": 1000,     # Updated min PWM - CRITICAL for correct movement
        "max": 1650,     # Updated max PWM - CRITICAL for correct movement
        # Expanded angle range with correct relationship to PWM values
        "min_angle": -75,  # Minimum angle (degrees)
        "max_angle": 75,   # Maximum angle (degrees)
    },
    "elbow": {
        "pin": 7,
        "rest": 800,
        "min": 800,      # Updated min PWM - CRITICAL for correct movement
        "max": 1650,     # Updated max PWM - CRITICAL for correct movement
        # Expanded angle range with correct relationship to PWM values
        "min_angle": -75,  # Minimum angle (degrees)
        "max_angle": 75,   # Maximum angle (degrees)
    },
    "wrist": {
        "pin": 6,
        "rest": 1500,
        "min": 600,
        "max": 2400,
        "min_angle": -90,
        "max_angle": 90,
    }
}