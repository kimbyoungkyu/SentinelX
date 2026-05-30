builtin_interfaces/Time stamp

string interceptor_id
string target_id

uint8 source
uint8 SOURCE_NONE=0
uint8 SOURCE_C2_TRACK=1
uint8 SOURCE_SEEKER=2
uint8 SOURCE_FUSED=3

bool valid

float32 relative_x_m
float32 relative_y_m
float32 relative_z_m

float32 relative_vx_mps
float32 relative_vy_mps
float32 relative_vz_mps

float32 range_m
float32 closing_speed_mps
float32 confidence
