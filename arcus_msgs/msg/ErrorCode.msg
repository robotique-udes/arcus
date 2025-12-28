# Source enum
uint16 GAP_FOLLOW = 0
uint16 PURE_PURSUIT = 1
uint16 CONTROLLER = 2
uint16 SLAM = 3
uint16 LIDAR = 4
uint16 VESC = 5
uint16 GUI = 6
uint16 MASTER = 7

# Error code enum
uint16 OK = 0
uint16 OFFLINE = 1
uint16 TIMEOUT = 2
uint16 INVALID_DATA = 3

std_msgs/Header header
uint16 source
uint16 error_code