/* 主机测试桩：只提供 arm.h 需要的类型，不含 HAL / FreeRTOS */
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
#include <stdint.h>
#include "fdcan.h"
#define CAN_FRAME_DATA_MAX 8U
#define pdPASS 1
typedef struct { int unused; } CanInstance_t;
typedef void (*CanRxHandler_t)(void *context, uint16_t id,
                               const uint8_t *data, uint8_t length);
int CANTransmit_Enqueue(FDCAN_HandleTypeDef *hcan, uint16_t std_id,
                        const uint8_t data[CAN_FRAME_DATA_MAX]);
int CAN_BindRx(CanInstance_t *instance, uint16_t id,
               CanRxHandler_t handler, void *context);
#endif
