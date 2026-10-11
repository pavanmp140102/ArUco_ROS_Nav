uint8 IDLE=0
uint8 SEARCH=1
uint8 APPROACH=2
uint8 FINAL=3
uint8 RECOVER=4
uint8 DOCKED=5
uint8 FAILED=6

std_msgs/Header header
uint8 state
int32 target_id                # -1 when idle
bool target_visible
float32 distance_to_dock       # d - front_offset - dock_distance; zero when docked
float32 lateral_error          # e_y, metres
float32 heading_error          # psi, radians
uint8 retries
