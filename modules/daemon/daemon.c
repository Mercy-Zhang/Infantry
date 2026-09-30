/**
 * @file daemon.c
 * @brief 模块看门狗实现
 */
#include "daemon.h"
#include <string.h>

static DaemonInstance daemon_pool[DAEMON_MX_CNT];
static uint8_t daemon_count = 0;

Daemon_Status_s daemon_status[DAEMON_MX_CNT];

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (daemon_count >= DAEMON_MX_CNT)
        return NULL;

    DaemonInstance *ins = &daemon_pool[daemon_count++];
    memset(ins, 0, sizeof(DaemonInstance));

    ins->reload_count = config->reload_count;
    ins->temp_count   = config->init_count;   /* 用 init_count 作初始值 */
    ins->callback     = config->callback;
    ins->owner_id     = config->owner_id;
    ins->alarm_silent = config->alarm_silent;
    ins->name         = config->name;

    return ins;
}

void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
    instance->reload_cnt++;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0 ? 1 : 0;
}

void DaemonTask(void)
{
    for (uint8_t i = 0; i < daemon_count; i++)
    {
        DaemonInstance *ins = &daemon_pool[i];

        if (ins->temp_count > 0)
        {
            ins->temp_count--;
            /* 边沿触发: 刚减到 0 时回调一次 */
            if (ins->temp_count == 0 && ins->callback != NULL)
                ins->callback(ins->owner_id);
        }

        daemon_status[i].name       = ins->name;
        daemon_status[i].online     = (ins->temp_count > 0) ? 1 : 0;
        daemon_status[i].reload_cnt = ins->reload_cnt;
    }
}
