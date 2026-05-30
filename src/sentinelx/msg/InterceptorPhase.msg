builtin_interfaces/Time stamp

string interceptor_id
string mission_id
string target_id

uint8 phase
uint8 PHASE_IDLE=0
uint8 PHASE_MISSION_LOADED=1
uint8 PHASE_READY_TO_LAUNCH=2
uint8 PHASE_LAUNCHED=3
uint8 PHASE_INERTIAL_MIDCOURSE=4
uint8 PHASE_SEEKER_SEARCH=5
uint8 PHASE_SEEKER_LOCKED=6
uint8 PHASE_TERMINAL_HOMING=7
uint8 PHASE_INTERCEPT_SUCCESS=8
uint8 PHASE_INTERCEPT_FAILED=9
uint8 PHASE_RETURN_HOME=10
uint8 PHASE_FAULT=11

uint8 authority
uint8 AUTHORITY_PRELAUNCH_C2=0
uint8 AUTHORITY_INERTIAL=1
uint8 AUTHORITY_SEEKER=2
uint8 AUTHORITY_SAFE_MODE=3

bool launched
bool terminal_phase
bool c2_command_allowed
bool c2_target_update_allowed
