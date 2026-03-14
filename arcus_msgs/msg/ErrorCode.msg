# Source enum
uint16 DISPARITY = 0
uint16 PURE_PURSUIT = 1
uint16 CONTROLLER = 2
uint16 SLAM = 3
uint16 LIDAR = 4
uint16 VESC = 5
uint16 GUI = 6
uint16 MASTER = 7
uint16 SAFETY = 8
uint16 PARTICLE_FILTER = 9

# Error code enum
uint16 OK = 0
uint16 OFFLINE = 1
uint16 TIMEOUT = 2
uint16 INVALID_DATA = 3
uint16 EMERGENCY_BRAKE = 4

std_msgs/Header header
uint16 source
uint16 error_code