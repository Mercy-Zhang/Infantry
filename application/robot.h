/**
 * @file robot.h
 * @brief 机器人初始化与主控制循环
 */
#ifndef INF_ROBOT_H
#define INF_ROBOT_H

void RobotInit(void);

/* 主控制循环(100Hz): gimbal → shoot → chassis */
void RobotTask(void);

#endif /* INF_ROBOT_H */
