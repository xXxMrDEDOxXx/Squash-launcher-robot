import rclpy

class LaunchParams:
    """
    Holds configuration parameters loaded at launch time for use by nodes.
    This could include target setpoints, file paths, topic names, etc.
    """
    # Define parameter names and default values as class attributes (for reference)
    DEFAULT_TARGET_PRESSURE = 100.0      # default target pressure (e.g., in kPa)
    DEFAULT_LOG_FILE = "pressure_log.csv" 
    DEFAULT_LOG_ENABLED = True
    DEFAULT_FLUSH_INTERVAL = 1.0        # seconds between periodic flushes
    DEFAULT_FLUSH_COUNT = 10           # number of entries to buffer before flush
    DEFAULT_ACTUAL_TOPIC = "/actual_pressure"
    DEFAULT_RECOMMENDED_TOPIC = "/recommended_pressure"

    def __init__(self, node: rclpy.node.Node):
        """
        Initialize LaunchParams by declaring and getting relevant ROS parameters from the given node.
        """
        # Declare parameters with defaults, then get their values
        self.target_pressure = node.declare_parameter(
            "target_pressure", LaunchParams.DEFAULT_TARGET_PRESSURE).value
        self.log_file = node.declare_parameter(
            "log_file", LaunchParams.DEFAULT_LOG_FILE).value
        self.logging_enabled = node.declare_parameter(
            "logging_enabled", LaunchParams.DEFAULT_LOG_ENABLED).value
        self.flush_interval = node.declare_parameter(
            "flush_interval", LaunchParams.DEFAULT_FLUSH_INTERVAL).value
        self.flush_count = node.declare_parameter(
            "flush_count", LaunchParams.DEFAULT_FLUSH_COUNT).value
        self.actual_topic = node.declare_parameter(
            "actual_pressure_topic", LaunchParams.DEFAULT_ACTUAL_TOPIC).value
        self.recommended_topic = node.declare_parameter(
            "recommended_pressure_topic", LaunchParams.DEFAULT_RECOMMENDED_TOPIC).value

        # (Additional parameters can be declared here as needed, e.g., physics constants)
    
    def __repr__(self):
        """For debugging: represent the current configuration as a string."""
        return (f"LaunchParams(target_pressure={self.target_pressure}, log_file='{self.log_file}', "
                f"logging_enabled={self.logging_enabled}, flush_interval={self.flush_interval}, "
                f"flush_count={self.flush_count}, actual_topic='{self.actual_topic}', "
                f"recommended_topic='{self.recommended_topic}')")
