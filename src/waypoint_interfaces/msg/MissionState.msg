# Mission State Message
# Tracks the current state of an inspection mission

std_msgs/Header header

# Mission identification
string rack_id                       # ID of the rack being inspected
string mission_id                    # Unique mission identifier

# Progress tracking
uint32 total_waypoints               # Total waypoints in mission
uint32 current_waypoint_index        # Current waypoint (0-indexed)
float32 percentage_complete          # Overall completion percentage

# Position tracking
geometry_msgs/Point takeoff_position # Home/launch position
geometry_msgs/Point current_position # Current drone position
geometry_msgs/Point last_waypoint    # Last completed waypoint

# Battery and system info
float32 battery_percentage           # Current battery level
bool system_healthy                  # Overall system health

# State machine
uint8 STATE_IDLE = 0
uint8 STATE_RUNNING = 1
uint8 STATE_PAUSED = 2
uint8 STATE_RTL = 3
uint8 STATE_COMPLETED = 4
uint8 STATE_ERROR = 5
uint8 state

# Timestamps
builtin_interfaces/Time mission_start_time
builtin_interfaces/Time last_update_time

# Resumable flag
bool can_resume                      # True if mission can be resumed
