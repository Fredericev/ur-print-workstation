uint8 STATE_UNKNOWN=0
uint8 STATE_READY=1
uint8 STATE_NOT_READY=2
uint8 STATE_UNREACHABLE=3

builtin_interfaces/Time stamp
uint8 state
bool moonraker_reachable
bool printer_ready
string printer_state
string detail
string[] manual_steppers
# Native extrusion contract, kept separate from the lock motor inventory.
string extrusion_protocol
string[] extruders
bool extruder_can_extrude
float64 extruder_temperature
float64 extruder_target
bool action_active
string active_operation_id
