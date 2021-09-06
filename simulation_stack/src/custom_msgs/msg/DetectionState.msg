# 前方传感器信息
# 接触到障碍物
uint8 COLLIDED = 0
# 靠近障碍物
uint8 NEAR_OBSTACLE = 1
uint8 NO_OBSTACLE = 2

# 侧方传感器信息
uint8 ALONG_WALL = 0
uint8 AWAY_FROM_WALL = 1
uint8 NO_WALL = 2

Header header
uint8 heading
uint8 heading_left
uint8 heading_right
uint8 left
uint8 right

float32 heading_range
float32 heading_left_range
float32 heading_right_range
float32 left_range
float32 right_range