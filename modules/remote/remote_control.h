/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 需要哪个协议解开对应注释即可(二选一)
#define DBUS  // DBUS: DJI DR16/DT7, 18 字节帧
// #define SBUS  // SBUS: 天地飞 ET16S 的 W.BUS(兼容 Futaba S.BUS), 25 字节帧

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ------------------- SBUS (天地飞 ET16S / W.BUS) ------------------------ */
// 一帧 25 字节: 0x0F + 22 字节通道数据 + 1 字节 flags + 0x00
#define SBUS_FRAME_SIZE 25u
// 16 个比例通道, 每个 11bit, 原始分辨率 0~2047 (中心 1024)
#define SBUS_CHANNEL_NUM 16u
#define SBUS_CH_START 0x0Fu // 帧头
#define SBUS_CH_END 0x00u   // 帧尾
// 三段开关判定阈值(相对中心 1024 的偏移量): |raw-1024| > 该值视为拨到两端
#define SBUS_SW_THRESHOLD 500

// 满行程归一化到的摇杆值(与旧 DBUS 代码的 ±660 约定保持一致, 下游增益无需重调)
#define SBUS_JOY_MAX 660
// 摇杆死区: 归一化到 ±660 后, |v| 小于该值一律置 0
// (消除回中抖动与量化噪声; 实测中性点偏移约 7, 取 15 覆盖偏移+抖动, 防云台/底盘漂移)
// 后续若把每通道 center 标定准了(中性点归 0), 可再调回 5~8
#define SBUS_JOY_DEADBAND 15
// 注意: 四个摇杆实测范围各不相同, 中心/最小/最大不能共用一组值;
// 已按通道分开标定, 数值在 remote_control.c 的 sbus_ch_calib[] 表里填写.

/* ===== 摇杆统一对称模式(推荐) =====
 * 1: 四个比例通道共用同一个"有效半行程", 输出曲线完全一致且对称:
 *    中心仍用各通道实测值(吸收物理回中偏差), 增益统一为 660/有效半行程;
 *    原始值超出有效范围的(如油门杆行程更大)输出钳位到 ±660(末端饱和).
 * 0: 按 sbus_ch_calib[] 各通道自己的 min/max 满行程映射(各自推满=±660, 但增益不同).
 * 有效半行程取值 = min(各通道 min(|center-min|, |max-center|)) 的公共交集.
 * 本机实测: ch[0]±903 ch[1]±905 ch[2]±1006 ch[3]±905 → 交集 903, 双向都不裁边. */
#define SBUS_JOY_UNIFORM 1
#define SBUS_JOY_UNIFORM_HALF_RANGE 903u

/* ================= 通道映射(单一真值源) =================
 * 数字是 ch[] 下标(= 通道号 - 1), ch[0]=CH1 ... ch[15]=CH16.
 * 摇杆(已实测确认):
 *   右手: 水平=ch[0](CH1)  竖直=ch[1](CH2)
 *   左手: 水平=ch[3](CH4)  竖直=ch[2](CH3)
 * 开关(三段: 上=2030 / 中=1024 / 下=17; 两段: 上=2030 / 下=17):
 *   SA=ch[4](CH5)   SB=ch[5](CH6)   SC=ch[6](CH7)
 *   SD=ch[7](CH8)   SE=ch[8](CH9)   SF=ch[9](CH10, 两段)
 * 功能分配:
 *   左手 → 云台(水平=yaw, 竖直=pitch);  右手 → 底盘(水平=横移, 竖直=前后)
 *   急停=SF(两段): 下=急停, 上=使能
 *   模式=SE(三段): 下=遥控器, 中=视觉, 上=小陀螺
 *   摩擦轮=SA(三段): 上=开摩擦轮, 中/下=待机
 *   拨盘=SB(三段): 下=待机, 中=单发, 上=连发(拨盘需摩擦轮先到位)
 * 注意: 若遥控器开了"联动/混控", 一个开关会同时动多个通道;
 *       每个宏只认其中一个通道, 被联动的重复通道直接忽略即可. */
#define SBUS_CH_ROCKER_L_ 3 // 左手水平 = CH4
#define SBUS_CH_ROCKER_L1 2 // 左手竖直 = CH3
#define SBUS_CH_ROCKER_R_ 0 // 右手水平 = CH1
#define SBUS_CH_ROCKER_R1 1 // 右手竖直 = CH2
#define SBUS_CH_SW_ESTOP 9     // 急停开关 = SF = ch[9](CH10, 两段)
#define SBUS_CH_SW_MODE  8     // 模式开关 = SE = ch[8](CH9, 三段)
#define SBUS_CH_SW_FRICTION 4  // 摩擦轮开关 = SA = ch[4](CH5, 三段)
#define SBUS_CH_SW_LOADER   5  // 拨盘开关 = SB = ch[5](CH6, 三段)
#define SBUS_CH_DIAL 11        // 旋钮/拨轮 = ch[11](CH12) 暂未启用(占位)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)3)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)2)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)1) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度
typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        uint16_t ch[16];   // SBUS 16 通道原始值(0~2047), ch[0]=CH1 ... ch[15]=CH16
        int16_t rocker_l_; // 左摇杆水平, -660~660
        int16_t rocker_l1; // 左摇杆竖直, -660~660
        int16_t rocker_r_; // 右摇杆水平, -660~660
        int16_t rocker_r1; // 右摇杆竖直, -660~660
        int16_t dial;      // 拨轮/旋钮, -660~660 (暂未启用)
        int16_t switch_right;
        int16_t switch_left;
        uint8_t switch_estop;    // 急停开关 (RC_SW_UP/DOWN, 两段)
        uint8_t switch_mode;     // 模式开关 (RC_SW_UP/MID/DOWN, 三段)
        uint8_t switch_friction; // 摩擦轮开关 SA (RC_SW_UP/MID/DOWN, 三段)
        uint8_t switch_loader;   // 拨盘开关 SB (RC_SW_UP/MID/DOWN, 三段)
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/* ---------------- 摇杆自动标定(死区+范围) ---------------- */
// 置 1 编译进标定功能(仅 SBUS 模式): 上电后调用 RemoteControlStartCalib() 启动,
// 按 RTT 提示先回中 1 秒、再满行程推拉 2 秒, 结束后会打印可直接粘贴进
// remote_control.c 里 sbus_ch_calib[] 的标定表格. 标定完成填表后置 0 即可.
#define SBUS_CALIB_ENABLE 1

/**
 * @brief 启动摇杆自动标定(死区+范围), 标定结果通过 RTT 日志打印
 *
 * @attention 需在 RemoteControlInit 之后且接收机正常出数据时调用;
 *            标定期间四个摇杆按提示操作, 油门杆(无回中弹簧)请停在
 *            你想要作为"零速/中立"的位置.
 */
void RemoteControlStartCalib(void);

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

#endif
