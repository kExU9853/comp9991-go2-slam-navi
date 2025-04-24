#!/usr/bin/env python3
"""
Unitree Go2 Robot Controller Node for ROS2
This module provides an interface between ROS2 Nav2 navigation stack and Unitree Go2 quadruped robot.

Core functionality:
1. Receives standard ROS2 `cmd_vel` messages from Nav2 navigation stack
2. Converts these motion commands into Unitree Go2 robot API's specific format
3. Provides additional service interfaces to control special robot actions (stand, sit, dance, etc.)

This controller acts as a bridge layer that enables the use of standard ROS2 navigation 
capabilities to control the Unitree Go2 quadruped robot's movements.
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from unitree_api.msg import Request
import json


class UnitreeRobotController(Node):
    """
    Controller for Unitree Go2 quadruped robot that processes velocity commands
    from Nav2 navigation stack and various action services.
    
    This node acts as a bridge between ROS2 Nav2 commands and the Unitree API,
    translating standard ROS Twist messages into robot-specific commands.
    The controller enables navigation capabilities on the quadruped robot.
    """
    
    # API command identifiers for different robot actions， copy from unitree api
    API_COMMANDS = {
        "idle": 0,
        "damping": 1001,
        "stand": 1004,
        "sit": 1005, 
        "recover": 1006,
        "move": 1008,
        "dance_a": 1022,
        "dance_b": 1023
    }
    
    # Logging level mapping
    LOG_LEVELS = {
        "debug": LoggingSeverity.DEBUG,
        "info": LoggingSeverity.INFO,
        "warn": LoggingSeverity.WARN,
        "error": LoggingSeverity.ERROR,
        "fatal": LoggingSeverity.FATAL
    }

    def __init__(self):
        """
        Initialize the robot controller node with parameters, publishers,
        subscribers, services, and timers.
        """
        # Initialize the node with a descriptive name
        super().__init__("unitree_go2_controller")
        
        # Load and configure node parameters
        self._initialize_parameters()
        
        # Set up communication channels
        self._setup_communication()
        
        # Initialize state variables
        self._velocity_timeout_count = 0.0
        self._current_cmd = None
        self._should_reset = False
        
        # Log successful initialization
        self.get_logger().info("Unitree Go2 Controller initialized successfully")

    def _initialize_parameters(self):
        """
        Declare and initialize all node parameters and configure logging level.
        """
        # Declare node parameters with default values
        self.declare_parameter("update_frequency", 200.0)  # Hz
        self.declare_parameter("velocity_timeout", 0.25)   # seconds
        self.declare_parameter("logging_level", "info")    # Default log level
        
        # Retrieve parameter values
        self._update_freq = self.get_parameter("update_frequency").value
        self._velocity_timeout = self.get_parameter("velocity_timeout").value
        
        # Calculate time intervals based on parameters
        self._update_interval = 1.0 / self._update_freq
        
        # Configure logging level based on parameter
        log_level_name = self.get_parameter("logging_level").value.lower()
        log_level = self.LOG_LEVELS.get(log_level_name, LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)
        
        self.get_logger().debug(f"Parameters loaded: update_freq={self._update_freq}Hz, " 
                               f"velocity_timeout={self._velocity_timeout}s")

    def _setup_communication(self):
        """
        Set up all communication channels including publishers, subscribers, 
        services, and timers.
        """
        # Configure publisher for robot commands
        self._robot_cmd_publisher = self.create_publisher(
            Request, 
            "/api/sport/request", 
            10
        )
        
        # Configure subscriber for velocity commands from Nav2
        self._velocity_subscriber = self.create_subscription(
            Twist, 
            "cmd_vel", 
            self._handle_velocity_command, 
            10
        )
        
        # Configure action services
        self._configure_services()
        
        # Set up the main control loop timer
        self._control_timer = self.create_timer(
            self._update_interval, 
            self._update_control_loop
        )

    def _configure_services(self):
        """
        Configure all service servers for robot actions.
        These services enable direct control of robot behaviors like standing,
        sitting, and special movements.
        """
        # Dictionary mapping service names to their handler methods
        service_handlers = {
            "stand_up": self._handle_stand_request,
            "lay_down": self._handle_sit_request,
            "recover_stand": self._handle_recovery_request,
            "damping_mode": self._handle_damping_request,
            "perform_dance_1": self._handle_dance1_request,
            "perform_dance_2": self._handle_dance2_request
        }
        
        # Create all services
        self._services = {}
        for service_name, handler in service_handlers.items():
            self._services[service_name] = self.create_service(
                Empty, 
                service_name, 
                handler
            )
            self.get_logger().debug(f"Service registered: {service_name}")

    def _create_robot_command(self, command_id, params=None):
        """
        Create a robot command Request message with the specified API ID and parameters.
        
        Args:
            command_id (int): The API command ID for the robot
            params (str, optional): JSON string with command parameters
            
        Returns:
            Request: The formatted command message ready to be published
        """
        command = Request()
        command.header.identity.api_id = command_id
        if params:
            command.parameter = params
        return command

    def _handle_velocity_command(self, velocity: Twist):
        """
        Process incoming velocity commands from Nav2 and translate them to robot movement commands.
        
        This is the core function that enables navigation by converting ROS Twist messages
        (linear and angular velocities) into robot-specific movement commands for the
        Unitree Go2 quadruped robot.
        
        Args:
            velocity (Twist): Linear and angular velocity components from Nav2
        """
        # Log received command for debugging
        self.get_logger().debug(
            f"Nav2 velocity command: forward={velocity.linear.x:.2f}, "
            f"lateral={velocity.linear.y:.2f}, "
            f"rotation={velocity.angular.z:.2f}"
        )
        
        # Create movement parameters
        movement_params = {
            "x": velocity.linear.x,  # Forward/backward motion
            "y": velocity.linear.y,  # Lateral motion (left/right)
            "z": velocity.angular.z  # Rotational motion
        }
        
        # Convert parameters to JSON string for Unitree API
        json_params = json.dumps(movement_params)
        
        # Create and publish robot command
        robot_cmd = self._create_robot_command(
            self.API_COMMANDS["move"],
            json_params
        )
        
        self._robot_cmd_publisher.publish(robot_cmd)
        self._current_cmd = robot_cmd
        self._velocity_timeout_count = 0.0  # Reset timeout counter

    def _update_control_loop(self):
        """
        Main control loop executed at regular intervals to manage robot state.
        
        This method handles command timeout monitoring and state resets.
        It ensures the robot stops if no commands are received for a certain period,
        which is an important safety feature for navigation.
        """
        # Check if velocity commands have timed out
        if self._velocity_timeout_count >= self._velocity_timeout:
            # Stop robot movement when timeout occurs
            self._send_idle_command()
        else:
            # Increment timeout counter
            self._velocity_timeout_count += self._update_interval
        
        # Handle any pending state reset requests
        if self._should_reset:
            self._should_reset = False
            self._send_idle_command()

    def _send_idle_command(self):
        """
        Send an idle command to stop the robot's movement.
        This is crucial for safety to ensure the robot stops when
        no valid navigation commands are received.
        """
        idle_cmd = self._create_robot_command(self.API_COMMANDS["idle"])
        self._robot_cmd_publisher.publish(idle_cmd)
        self.get_logger().debug("Idle command issued to stop robot")

    def _send_action_command(self, action_name):
        """
        Helper method to send a robot action command and log it.
        
        Args:
            action_name (str): Name of the action to perform
        """
        if action_name in self.API_COMMANDS:
            cmd = self._create_robot_command(self.API_COMMANDS[action_name])
            self._robot_cmd_publisher.publish(cmd)
            self._should_reset = True
            self.get_logger().info(f"Action executed: {action_name}")
        else:
            self.get_logger().error(f"Unknown action requested: {action_name}")

    # Service callback handlers for different robot actions
    def _handle_stand_request(self, request, response):
        """Handle stand up service request."""
        self._send_action_command("stand")
        return response

    def _handle_sit_request(self, request, response):
        """Handle sit down (lay down) service request."""
        self._send_action_command("sit")
        return response

    def _handle_recovery_request(self, request, response):
        """Handle recovery stand service request."""
        self._send_action_command("recover")
        return response

    def _handle_damping_request(self, request, response):
        """Handle damping mode service request."""
        self._send_action_command("damping")
        return response

    def _handle_dance1_request(self, request, response):
        """Handle dance 1 service request."""
        self._send_action_command("dance_a")
        return response

    def _handle_dance2_request(self, request, response):
        """Handle dance 2 service request."""
        self._send_action_command("dance_b")
        return response


def main(args=None):
    """
    Main entry point of the Unitree Go2 controller node.
    
    This function initializes the ROS context, creates the controller node,
    and starts the event loop to process callbacks from Nav2 and service requests.
    
    Args:
        args: Command line arguments passed to ROS
    """
    # Initialize ROS context
    rclpy.init(args=args)
    
    # Create controller node
    controller = UnitreeRobotController()
    
    # Enter ROS event loop to process callbacks
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        controller.get_logger().info("Shutting down Unitree Go2 Controller")
        rclpy.shutdown()


if __name__ == "__main__":
    main()