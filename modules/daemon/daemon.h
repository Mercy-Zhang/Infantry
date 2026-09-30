/**
 * @file daemon.h
 * @brief 模块看门狗(离线检测)
 */
#ifndef INF_DAEMON_H
#define INF_DAEMON_H

#include <stdint.h>

#define DAEMON_MX_CNT 64

typedef void (*offline_callback)(void *);

typedef struct daemon_ins
{
    uint16_t reload_count;     /* 喂狗重载值 */
    offline_callback callback; /* 离线时调用一次 */

    uint16_t temp_count;       /* 当前值, 减到 0 即离线 */
    void *owner_id;            /* 拥有者的实例地址 */
    uint8_t alarm_silent;      /* 1=离线不报警 */
    const char *name;          /* 模块名(诊断) */
    uint32_t reload_cnt;       /* 累计喂狗次数(诊断) */
} DaemonInstance;

typedef struct
{
    const char *name;
    uint8_t online;
    uint32_t reload_cnt;
} Daemon_Status_s;

extern Daemon_Status_s daemon_status[DAEMON_MX_CNT];

typedef struct
{
    uint16_t reload_count;
    uint16_t init_count;       /* 上线等待: 初始 temp_count */
    offline_callback callback;
    void *owner_id;
    uint8_t alarm_silent;
    const char *name;
} Daemon_Init_Config_s;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);

/* 周期调用(建议 100Hz): 递减计数, 触发离线回调 */
void DaemonTask(void);

#endif /* INF_DAEMON_H */
