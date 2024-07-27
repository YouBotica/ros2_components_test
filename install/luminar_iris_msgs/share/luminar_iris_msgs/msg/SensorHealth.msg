builtin_interfaces/Time stamp

uint8 SYSTEM_STATUS_UNKNOWN = 0
uint8 SYSTEM_STATUS_STANDBY = 1
uint8 SYSTEM_STATUS_ACTIVE = 2
uint8 SYSTEM_STATUS_SHUTDOWN = 3

float32 battery_voltage
float32 system_voltage
float32 system_temperature
uint8 system_mode
bool system_ok
bool laser_ok
bool scanner_ok
bool receiver_ok
bool datapath_ok