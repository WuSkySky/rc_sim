# LED detection result for one processed camera frame.
std_msgs/Header header

bool valid
bool[] led_states  # Empty when valid is false.
string reason      # Empty when valid is true.
