#pragma once
#include "dil/can.h"
#include "main.h"

typedef struct {
    uint32_t timeStamp_ms;
    uint8_t valveIndex;
    uint8_t reserved[3];
} ValveCmdPacketData;

typedef union {
	ValveCmdPacketData fields;
	uint8_t data[sizeof(ValveCmdPacketData)];
} ValveCmdPayload;

typedef struct {
    CANHeader header;
    ValveCmdPayload payload;
} ValveCmdPacket;

// Build a packet
static inline void valveCmdPacketMake(CanValveIndex valve, CanValveCmd cmd, ValveCmdPacket* packet)
{
    packet->header.frame.targetID = CAN_NODE_ECU;
    packet->header.frame.senderID = CAN_NODE_FCU;
    packet->header.frame.deviceState = cmd;
    packet->header.frame.messageID = CAN_ID_CMD_VALVE;
    packet->header.frame.errorCtrl = 0;
    packet->header.frame.errorCode = 0;
    packet->payload.fields.valveIndex = valve;
    packet->payload.fields.timeStamp_ms = HAL_GetTick();
}
static inline void valveCmdPacketParse(uint32_t extId,
                                       const uint8_t* rxData,
                                       CanValveIndex* outValve,
                                       CanValveCmd*   outCmd,
                                       uint32_t*      outTimestamp)
{
    CANHeader h;
    h.code = extId;

    const ValveCmdPayload* p = (const ValveCmdPayload*)rxData;

    *outValve     = (CanValveIndex)p->fields.valveIndex;
    *outCmd       = (CanValveCmd)h.frame.deviceState;
    *outTimestamp = p->fields.timeStamp_ms;
}