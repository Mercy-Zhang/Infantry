/**
 * @file remote.h
 * @brief 遥控器适配层(SBUS/DBUS → g_input.remote)
 */
#ifndef INF_REMOTE_H
#define INF_REMOTE_H

void RemoteInit(void);

/* 周期调用: 把 rc_data 映射到 g_input.remote */
void RemoteTask(void);

#endif /* INF_REMOTE_H */
