from dataclasses import dataclass
from datetime import datetime

@dataclass
class LogEntry:
    """Data structure for a single log entry of pressures."""
    timestamp: float         # ROS time (or system time) when the entry was recorded (in seconds)
    actual_pressure: float   # actual pressure value at that time
    recommended_pressure: float  # recommended pressure value at that time

    def to_csv(self) -> str:
        """Return a CSV-formatted string of the log entry."""
        # Format: ISO timestamp for readability, actual, recommended
        timestr = datetime.fromtimestamp(self.timestamp).isoformat()
        return f"{timestr}, {self.actual_pressure:.3f}, {self.recommended_pressure:.3f}\n"
