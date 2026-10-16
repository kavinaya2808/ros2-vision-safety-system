# State of the safety monitor, published on every control cycle.

std_msgs/Header header

uint8 STATE_INIT=0            # no detection received yet: stop (fail-safe default)
uint8 STATE_CLEAR=1           # fresh perception, path clear: commands pass through
uint8 STATE_STOP_OBSTACLE=2   # obstacle in danger zone: forward motion blocked
uint8 STATE_STOP_STALE=3      # perception too old or missing: full stop

uint8 state
string reason                 # human-readable explanation of the current state

float64 detection_age_s       # now - stamp of the newest detection (inf if none)
bool command_timed_out        # no fresh /cmd_vel_raw: output forced to zero

geometry_msgs/Twist requested # last command received from upstream
geometry_msgs/Twist commanded # command actually sent to the robot
