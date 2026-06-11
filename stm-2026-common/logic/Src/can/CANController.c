#include "can/CANController.h"
#include "dil/can_bus.h"

void CANController_Init(CANController *ctrl, const CANControllerConfig *cfg)
{
    ctrl->cfg = cfg;
}

bool CANController_Process(CANController *ctrl)
{
    CANHeader header;
    uint8_t rxData[8];

    if (!CanBus_Receive(&header, rxData)) return false;  /* ring empty */

    /* Frame was dequeued; report true even if it is not ours or unhandled,
       so the caller can keep draining the ring. */
    if (header.frame.targetID != ctrl->cfg->nodeID) return true;

    const CANHandlerEntry *h = ctrl->cfg->handlers;
    for (uint8_t i = 0; i < ctrl->cfg->handlerCount; i++, h++)
    {
        if (h->messageID == header.frame.messageID)
        {
            h->handler(h->ctx, &header, rxData);
            break;
        }
    }
    return true;
}