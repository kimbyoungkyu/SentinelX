builtin_interfaces/Time stamp

string interceptor_id
int32 mavlink_sys_id

uint8 STATUS_UNKNOWN=0
uint8 STATUS_READY=1
uint8 STATUS_ASSIGNED=2
uint8 STATUS_LAUNCHING=3
uint8 STATUS_TRACKING=4
uint8 STATUS_ENGAGING=5
uint8 STATUS_RETURNING=6
uint8 STATUS_LANDED=7
uint8 STATUS_CHARGING=8
uint8 STATUS_OFFLINE=9
uint8 STATUS_LOST=10
uint8 STATUS_ERROR=11
uint8 status

uint8 MODE_UNKNOWN=0
uint8 MODE_MANUAL=1
uint8 MODE_STABILIZED=2
uint8 MODE_POSITION=3
uint8 MODE_MISSION=4
uint8 MODE_OFFBOARD=5
uint8 MODE_RETURN=6
uint8 MODE_LAND=7
uint8 MODE_HOLD=8
uint8 mode

bool armed
bool connected
bool offboard_available

float32 battery_percent
float32 battery_voltage

GeoPosition position
VelocityNED velocity

string mission_id
string target_id
string message