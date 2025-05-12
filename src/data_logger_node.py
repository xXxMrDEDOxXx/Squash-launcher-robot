import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # using standard service to toggle logging
# Import our custom classes from common and logger packages
# (Assume the proper Python package structure where these imports work)
from common.launch_params import LaunchParams
from common.physics_calculator import PhysicsCalculator
from logger.log_entry import LogEntry

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        # Load configuration parameters
        self.params = LaunchParams(self)
        self.get_logger().info(f"DataLoggerNode initialized with params: {self.params}")

        # Initialize PhysicsCalculator (if needed for any data processing; optional here)
        self.physics_calc = PhysicsCalculator(self.params)

        # Prepare to open the log file
        file_mode = 'w'  # overwrite by default on each run (could be 'a' to append if desired)
        self.log_file_handle = open(self.params.log_file, file_mode)
        # Write CSV header for clarity
        self.log_file_handle.write("timestamp, actual_pressure, recommended_pressure\n")
        self.log_file_handle.flush()

        # Logging state
        self.logging_enabled = self.params.logging_enabled
        self.log_buffer = []   # buffer to store LogEntry objects not yet flushed

        # Subscribe to topics
        self.last_actual = None
        self.last_recommended = None
        self.sub_actual = self.create_subscription(
            # Assuming actual pressure comes as a Float64 message (std_msgs.msg.Float64)
            # If a custom message or sensor_msgs/FluidPressure were used, adjust import and type.
            rclpy.qos.QoSProfile,  # Placeholder: will replace with actual message type below
            self.params.actual_topic,
            self._actual_callback,
            10  # queue size
        )
        self.sub_recommended = self.create_subscription(
            rclpy.qos.QoSProfile,  # Placeholder: actual message type to be imported or defined
            self.params.recommended_topic,
            self._recommended_callback,
            10
        )
        # Note: Replace rclpy.qos.QoSProfile with actual message type, e.g., std_msgs.msg.Float64.

        # Create a timer for periodic flushing of logs to disk (to avoid too infrequent writes)
        self.flush_timer = self.create_timer(
            self.params.flush_interval, 
            self._flush_logs_callback
        )

        # Create a service for toggling logging on/off
        self.toggle_service = self.create_service(
            SetBool, 
            'set_logging', 
            self._handle_set_logging
        )
        self.get_logger().info("DataLoggerNode ready. Logging is " +
                               ("ENABLED" if self.logging_enabled else "DISABLED"))
    
    def _actual_callback(self, msg):
        """Callback for actual pressure topic."""
        if not self.logging_enabled:
            return  # if logging disabled, ignore incoming data
        # Extract the actual pressure value from the message
        actual_val = msg.data if hasattr(msg, 'data') else msg  # adapt if msg is plain number
        self.last_actual = float(actual_val)
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1e-9
        # If we have a recent recommended value, log an entry combining them
        if self.last_recommended is not None:
            entry = LogEntry(timestamp, self.last_actual, self.last_recommended)
            self.log_buffer.append(entry)
            # If buffer reached threshold, flush it
            if len(self.log_buffer) >= self.params.flush_count:
                self._flush_buffer_to_file()
        # (If no recommended value yet, we wait until at least one recommended reading arrives to log a pair)
    
    def _recommended_callback(self, msg):
        """Callback for recommended pressure topic."""
        # Similar to actual callback
        if not self.logging_enabled:
            return
        rec_val = msg.data if hasattr(msg, 'data') else msg
        self.last_recommended = float(rec_val)
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec*1e-9
        if self.last_actual is not None:
            entry = LogEntry(timestamp, self.last_actual, self.last_recommended)
            self.log_buffer.append(entry)
            if len(self.log_buffer) >= self.params.flush_count:
                self._flush_buffer_to_file()
        # (If no actual value yet, wait until actual arrives. This way we log only when both values are known at least once.)

    def _flush_logs_callback(self):
        """Periodic timer callback to flush buffered logs to file."""
        if not self.logging_enabled:
            return
        if self.log_buffer:
            # Write all buffered entries to file
            self._flush_buffer_to_file()
    
    def _flush_buffer_to_file(self):
        """Helper to write all entries in the buffer to disk and clear the buffer."""
        for entry in self.log_buffer:
            self.log_file_handle.write(entry.to_csv())
        # Ensure data is written to disk (flush)
        self.log_file_handle.flush()
        # Clear the buffer after flushing
        self.log_buffer.clear()
        self.get_logger().debug("Flushed log buffer to file.")
    
    def _handle_set_logging(self, request, response):
        """Service callback to enable/disable logging."""
        enable = request.data  # boolean from SetBool request
        if enable and not self.logging_enabled:
            # Enabling logging
            self.logging_enabled = True
            self.get_logger().info("Logging ENABLED via service call.")
        elif not enable and self.logging_enabled:
            # Disabling logging
            self.logging_enabled = False
            self.get_logger().info("Logging DISABLED via service call.")
            # Optionally flush any remaining data when logging is turned off
            self._flush_buffer_to_file()
        # If request is to enable when already enabled or disable when already disabled, just acknowledge
        response.success = True
        response.message = ("Logging enabled" if enable else "Logging disabled")
        return response

    def destroy_node(self):
        """Override destroy_node to ensure file is closed on shutdown."""
        # Flush any remaining logs before shutting down
        if self.log_buffer:
            self._flush_buffer_to_file()
        self.log_file_handle.close()
        self.get_logger().info("Log file closed. DataLoggerNode shutting down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DataLoggerNode interrupted by keyboard.")
    finally:
        # Destroy node explicitly (calls destroy_node)
        node.destroy_node()
        rclpy.shutdown()
