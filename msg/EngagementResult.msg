builtin_interfaces/Time stamp

string mission_id
string interceptor_id
string target_id

uint8 result

uint8 UNKNOWN=0
uint8 SUCCESS_SIM=1
uint8 MISSED_SIM=2
uint8 ABORTED=3
uint8 TARGET_LOST=4
uint8 SYSTEM_FAULT=5
uint8 SAFETY_ABORT=6

float32 final_distance
float32 engagement_time_sec

string summary