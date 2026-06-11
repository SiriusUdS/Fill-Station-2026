#pragma once
#include "dil/can_types.h"   /* HAL-free CAN types (usable on the M7) */
#include "stm32h7xx_hal.h"   /* HAL_GetTick() */

typedef struct {
    uint32_t timeStamp_ms;
    uint8_t valveIndex;
    uint8_t reserved[3];
} ValveStatusPacketData;

typedef union {
	ValveStatusPacketData fields;
	uint8_t data[sizeof(ValveStatusPacketData)];
} ValveStatusPayload;

typedef struct {
    CANHeader header;
    ValveStatusPayload payload;
} ValveStatusPacket;

// Build a status packet
static inline void valveStatusPacketMake(uint8_t sender, uint8_t target,
                                         CanValveIndex valve, CanValveStatus status,
                                         ValveStatusPacket* packet)
{
    packet->header.frame.targetID = sender;
    packet->header.frame.senderID = target;
    packet->header.frame.deviceState = status;
    packet->header.frame.messageID = CAN_ID_STATUS_VALVE;
    packet->header.frame.errorCtrl = 0;
    packet->header.frame.errorCode = 0;
    packet->payload.fields.valveIndex = valve;
    packet->payload.fields.timeStamp_ms = HAL_GetTick();
}

static inline void valveStatusPacketParse(uint32_t extId,
                                          const uint8_t* rxData,
                                          CanValveIndex*  outValve,
                                          CanValveStatus* outStatus,
                                          uint32_t*       outTimestamp)
{
    CANHeader h;
    h.code = extId;

    const ValveStatusPayload* p = (const ValveStatusPayload*)rxData;

    *outValve     = (CanValveIndex) p->fields.valveIndex;
    *outStatus    = (CanValveStatus)h.frame.deviceState;
    *outTimestamp = p->fields.timeStamp_ms;
}