# dSPACE MPC가 10ms마다 전송하는 odom 기준 XY 예측 궤적.
# DBC v3의 TRAJ_P00_XY..TRAJ_P50_XY 한 세트와 1:1 대응한다.
std_msgs/Header header
float32[51] x
float32[51] y
