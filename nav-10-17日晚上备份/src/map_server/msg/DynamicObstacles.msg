# Current measured obstacles only; no trajectory prediction.
# data uses raw Costmap uint8 values (255=free or 0=obstacle), never occupancy probabilities.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
nav_msgs/MapMetaData info
uint8[] data
float32[] confidence
bool current
string status
