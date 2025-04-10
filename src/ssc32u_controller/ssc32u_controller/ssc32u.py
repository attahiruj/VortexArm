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

    def move_multiple_servos(self, positions, speed=None, time=None):
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

            if position < self.min_pwm or position > self.max_pwm:
                raise ValueError(
                    f"Position must be between {self.min_pwm} & {self.max_pwm}"
                    )

            # Add servo command to the command string
            command += f"#{channel}P{position}"

        if speed is not None:
            command += f"S{speed}"

        if time is not None:
            command += f"T{time}"

        print(f"Command to send: {command}")

        return self.send_command(command)

