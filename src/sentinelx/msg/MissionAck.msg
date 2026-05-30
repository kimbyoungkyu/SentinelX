builtin_interfaces/Time stamp

string command_id
string mission_id
string interceptor_id

bool accepted
uint8 result_code

uint8 OK=0
uint8 REJECTED=1
uint8 BUSY=2
uint8 INVALID_TARGET=3
uint8 NOT_READY=4
uint8 SAFETY_BLOCKED=5
uint8 INTERNAL_ERROR=6

string message