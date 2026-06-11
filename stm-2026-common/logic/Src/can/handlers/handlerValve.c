#include "can/handlers/handlerValve.h"
#include "can/packets/ValveCmdPacket.h"
#include "can/packets/ValveStatusPacket.h"
#include "dil/can_bus.h"

void handler_valve(void *ctx, const CANHeader *header, const uint8_t *rxData)
{
    ValveHandlerCtx *vc = (ValveHandlerCtx *)ctx;

    CanValveIndex valve; CanValveCmd cmd; uint32_t ts;
    valveCmdPacketParse(header->code, rxData, &valve, &cmd, &ts);

    uint8_t idx = (uint8_t)valve;
    if (idx >= vc->valveCount) return;

    Valve *v = &vc->valves[idx];
    if      (cmd == CAN_CMD_OPEN)  valveOpen(v);
    else if (cmd == CAN_CMD_CLOSE) valveClose(v);

    CanValveStatus    status = (v->state == VALVE_STATE_OPEN)
                             ? CAN_STATUS_OPEN : CAN_STATUS_CLOSED;
    ValveStatusPacket pkt;
    valveStatusPacketMake(header->frame.targetID,
        header->frame.senderID,
        valve,
        status,
        &pkt);
    CanBus_Send(pkt.header.code, pkt.payload.data);
}