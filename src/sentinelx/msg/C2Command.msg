builtin_interfaces/Time stamp

string command_id
string mission_id
string interceptor_id
string target_id

uint8 command_type

uint8 ASSIGN_TARGET=1
uint8 PREPARE_INTERCEPT=2
uint8 AUTHORIZE_LAUNCH=3
uint8 START_INTERCEPT=4
uint8 UPDATE_TARGET=5
uint8 UPDATE_MISSION=6
uint8 HOLD=7
uint8 ABORT=8
uint8 RETURN_HOME=9
uint8 LAND=10

string reason