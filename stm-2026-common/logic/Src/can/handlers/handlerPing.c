#include "can/handlers/handlerPing.h"
#include "dil/can_bus.h"

void handler_ping(void *ctx, const CANHeader *header, const uint8_t *rxData)
{
    uint8_t senderID = *(uint8_t *)ctx;

    CANHeader resp = {0};
    resp.frame.senderID = senderID;
    resp.frame.targetID = header->frame.senderID;
    resp.frame.messageID = CAN_ID_COMM_PONG;
    CanBus_Send(resp.code, rxData);
}