builtin_interfaces/Time stamp

string interceptor_id
string mission_id

uint8 vehicle_state

uint8 IDLE=0
uint8 READY=1
uint8 ARMED=2
uint8 ACTIVE=3
uint8 HOLDING=4
uint8 RETURNING=5
uint8 LANDED=6
uint8 FAULT=7

float64 latitude
float64 longitude
float32 altitude

float32 velocity_x
float32 velocity_y
float32 velocity_z

float32 battery_remaining
bool armed
bool offboard_enabled
bool healthy