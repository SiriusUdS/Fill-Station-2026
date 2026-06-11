#pragma once

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* CAN protocol definitions shared by both cores (HAL-independent).          */
/* The FDCAN driver API that needs FDCAN_HandleTypeDef lives in can.h, which */
/* is M4-only; the M7 controller includes just this file.                    */
/* ------------------------------------------------------------------------- */

/* CAN extended-identifier layout (29-bit), overlaid on a 32-bit word        */
typedef struct {
    uint32_t senderID:4;
    uint32_t targetID:4;
    uint32_t deviceState:4;
    uint32_t messageID:8;
    uint32_t errorCtrl:2;
    uint32_t errorCode:7;
    uint32_t reserved:3;
}FrameCANHeader;

typedef union {
    FrameCANHeader frame;
    uint32_t code;
} CANHeader;

// Node identity uses the system-wide board ids (single source of truth) from
// sirius-headers-common/Telecommunication/PacketHeaderVariable.h:
//   FILLING_STATION_BOARD_ID (FCU), ENGINE_BOARD_ID (ECU), GS_CONTROL_BOARD_ID,
//   BOARD_BROADCAST_ID. The old CAN_NODE_* enum was removed — it numbered the
//   boards differently from the board ids, which was a cross-transport hazard.

// Id du message
typedef enum {
    CAN_ID_CMD_VALVE      = 0x01,   /* FCU → ECU : Change valve status        */
    CAN_ID_STATUS_VALVE   = 0x02,   /* ECU → FCU : Return valve status        */
    CAN_ID_COMM_PING      = 0x7E,   /* any → node     : communication test    */
    CAN_ID_COMM_PONG      = 0x7F,   /* node → sender  : reply, echoes payload  */
} CanMsgId;

typedef enum {
    CAN_VALVE_1 = 0,
    CAN_VALVE_2 = 1,
} CanValveIndex;

typedef enum {
    CAN_CMD_CLOSE = 0x00,
    CAN_CMD_OPEN  = 0x01,
} CanValveCmd;

// Status de la valve
typedef enum {
    CAN_STATUS_UNKNOWN  = 0x00,
    CAN_STATUS_OPEN     = 0x01,
    CAN_STATUS_OPENING  = 0x02,
    CAN_STATUS_CLOSED    = 0x03,
    CAN_STATUS_CLOSING  = 0x04,
    CAN_STATUS_FAULT = 0x05
} CanValveStatus;
