builtin_interfaces/Time stamp

string interceptor_id
string mission_id
string target_id

uint8 guidance_mode
uint8 GUIDANCE_NONE=0
uint8 GUIDANCE_HOLD=1
uint8 GUIDANCE_GPS_INS=2
uint8 GUIDANCE_SEEKER_TERMINAL=3
uint8 GUIDANCE_RETURN_HOME=4
uint8 GUIDANCE_LAND=5

float32 target_x_m
float32 target_y_m
float32 target_z_m

float32 velocity_x_mps
float32 velocity_y_mps
float32 velocity_z_mps

float32 yaw_rad

bool use_position
bool use_velocity
bool use_yaw
bool terminal_phase
