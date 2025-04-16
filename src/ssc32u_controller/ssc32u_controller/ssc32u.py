"""
SSC_32U Controller for controlling servos using SSC-32U board.
This module provides methods to connect to the SSC-32U,
send commands to move servos, and manage multiple servos simultaneously.

Author: Attahiru Jibril
Date: 2025-04-14
"""

import serial
import time


class SSC_32U:
    def __init__(self, port="COM3", baud_rate=9600, timeout=1):
        """Initialize SSC_32U controller with specified port and baud rate."""
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.last_command = None
        self.last_command_time = 0
        self.min_pwm = 600   # default is 500, but servo makes noise at 500
        self.max_pwm = 2400  # default is 2500, but servo makes noise at 2500

    def connect(self):
        """Connect to SSC_32U."""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Allow time for connection to establish
            print(f"Connected to SSC_32U on {self.port}")
            self.connected = True

            return True
        except Exception as e:
            print(f"Warning: Could not connect to SSC_32U: {e}")
            print("Continuing without SSC_32U control...")
            self.serial = None
            self.connected = False

            return False

    def disconnect(self):
        """Disconnect from SSC_32U."""
        if self.serial:
            self.serial.close()
            self.serial = None
            self.connected = False
            print("Disconnected from SSC_32U")

    def send_command(self, command):
        """
        Send command to SSC_32U.
        For most servo commands, no response is expected.
        """
        if not self.serial:
            return False

        try:
            # Ensure command ends with carriage return
            if not command.endswith('\r'):
                command += '\r'

            self.serial.write(command.encode())
            print(f"Sent command: {command.strip()}")
            self.last_command = command
            self.last_command_time = time.time()
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def move_servo(self, channel, position, speed=None, time=None):
        """
        Move a servo to a specific position.
        # <ch> P <pw> S <spd> T <time> <cr>

        Args:
            channel (int): Servo channel (0-31)
            position (int): Pulse width in microseconds (500-2500)
            speed (int, optional): Speed in microseconds per second
            time (int, optional): Time to complete move in milliseconds

        Returns:
            bool: True if command was sent successfully
        """
        if channel < 0 or channel > 31:
            raise ValueError("Channel must be between 0 and 31")

        if position < self.min_pwm or position > self.max_pwm:
            raise ValueError(
                f"Position must be between {self.min_pwm} & {self.max_pwm} ms")

        command = f"#{channel}P{position}"

        if speed is not None:
            command += f"S{speed}"

        if time is not None:
            command += f"T{time}"

        return self.send_command(command)

    def move_multiple_servos(self, positions, speed=None, time=None, angle=False):
        """
        Move multiple servos to specific positions.
        # <ch> P <pw> S <spd> ... # <ch> P <pw> S <spd> T <time> <cr>

        Args:
            positions (list): List of tuples (channel, position)
            speed (int, optional): Speed in microseconds per second
            time (int, optional): Time to complete move in milliseconds

        Returns:
            bool: True if command was sent successfully
        """
        if len(positions) > 32:
            raise ValueError("Cannot control more than 32 servos at once")

        command = ""
        for channel, position in positions:
            if channel < 0 or channel > 31:
                raise ValueError("Channel must be between 0 and 31")

            if angle:
                # Convert angle to PWM value
                position = self.map_deg_to_pwm(position)
            
            if position < self.min_pwm or position > self.max_pwm:
                raise ValueError(
                    f"Position must be between {self.min_pwm} & {self.max_pwm}"
                    )
            else:
                # Add servo command to the command string
                command += f"#{channel}P{position}"

        if speed is not None:
            command += f"S{speed}"

        if time is not None:
            command += f"T{time}"

        print(f"Command to send: {command}")

        return self.send_command(command)

    def query_command(self, command, wait_for_response=True):
        """
        Send a query command and wait for response.
        Returns the response string or None if no response.
        """
        if not self.serial:
            return None

        try:
            # Ensure command ends with carriage return
            if not command.endswith('\r'):
                command += '\r'

            self.serial.write(command.encode())
            print(f"Sent query: {command.strip()}")

            if wait_for_response:
                return self.read_response()
            return None
        except Exception as e:
            print(f"Error sending query: {e}")
            return None

    def read_response(self, timeout=1.0):
        """
        Read response from SSC_32U with timeout.
        Returns the response string or None if timeout or error.
        """
        if not self.serial:
            return None

        start_time = time.time()
        response_buffer = bytearray()

        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting > 0:
                try:
                    byte_data = self.serial.read(1)
                    response_buffer.extend(byte_data)

                    # Check if we've received a complete response (usually ends with CR)
                    if byte_data == b'\r' or len(response_buffer) > 0 and time.time() - start_time > 0.1:
                        try:
                            response = response_buffer.decode('utf-8').strip()
                            print(f"SSC_32U response: {response}")
                            return response
                        except UnicodeDecodeError:
                            print("Error: Unable to decode SSC_32U response")
                            return None
                except Exception as e:
                    print(f"Error reading from SSC_32U: {e}")
                    return None
            time.sleep(0.01)  # Small delay to prevent CPU hogging

        if len(response_buffer) > 0:
            try:
                response = response_buffer.decode('utf-8').strip()
                print(f"SSC_32U response (timeout reached): {response}")
                return response
            except:
                pass

        print("No response from SSC_32U")
        return None

    def query_movement_status(self):
        """
        Query if servos are still moving.

        Returns:
            str: "." if movement complete, "+" if in progress, None if error
        """
        return self.query_command("Q")

    def is_ready(self):
        """
        Check if SSC_32U is ready to receive commands.

        Returns:
            bool: True if ready, False otherwise
        """
        return self.query_movement_status() == "."

    def map_deg_to_pwm(self, angle):
        """
        Map a degree angle to PWM value.

        Args:
            angle (float): Angle in degrees

        Returns:
            int: Corresponding PWM value
        """

        # Map the angle to the PWM range (600-2400)
        pwm = int((angle + 90) * (self.max_pwm - self.min_pwm) / 180 + self.min_pwm)
        return pwm
