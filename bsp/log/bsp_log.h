/**
 * @file bsp_log.h
 * @brief 日志输出(基于 SEGGER RTT)
 */
#ifndef INF_BSP_LOG_H
#define INF_BSP_LOG_H

#include "SEGGER_RTT.h"

#define BUFFER_INDEX 0

#define LOGINFO(fmt, ...)    SEGGER_RTT_printf(0, "[INF] " fmt "\r\n", ##__VA_ARGS__)
#define LOGWARNING(fmt, ...) SEGGER_RTT_printf(0, "[WRN] " fmt "\r\n", ##__VA_ARGS__)
#define LOGERROR(fmt, ...)   SEGGER_RTT_printf(0, "[ERR] " fmt "\r\n", ##__VA_ARGS__)

#endif /* INF_BSP_LOG_H */
