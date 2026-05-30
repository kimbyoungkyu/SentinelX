builtin_interfaces/Time stamp

string mission_id
string interceptor_id
string target_id

uint8 phase

uint8 NONE=0
uint8 ASSIGNED=1
uint8 PREPARING=2
uint8 LAUNCHED=3
uint8 MIDCOURSE=4
uint8 COOPERATIVE_TRACKING=5
uint8 TERMINAL_APPROACH_SIM=6
uint8 COMPLETED=7
uint8 ABORTED=8
uint8 FAILED=9

float32 distance_to_target
float32 relative_speed
float32 mission_elapsed_sec

string status_text