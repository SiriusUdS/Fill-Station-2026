#include "sirius-headers-common/FillingStation/FillingStationState.h"
#include "sirius-headers-common/FillingStation/FillingStationErrorStatus.h"
#include "communication/ethernet/ethernet.h"
#include "SDCard.h"
#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "string.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "data_integrity/crc.h"
#include "dil/can_bus.h"
#include "dil/can_types.h"
#include "can/CANController.h"
#include "can/handlers/handlerPing.h"
#include "can/packets/ValveCmdPacket.h"
#include "can/packets/ValveStatusPacket.h"

#define RX_WATCHDOG 500

typedef struct {
    uint8_t fillState;
    uint32_t currentTick;
    uint32_t lastEthernetMessageTx_MS;
    uint32_t lastEthernetMessageRx_MS;
    FillingStationErrorStatus errors;
    NetAddr gsAddr;
    SDCard sd;
    /* Latest CAN frame received on FDCAN1. */
    uint32_t canRxCount;
    uint32_t canLastRxId;
    uint8_t  canLastRxData[8];
} FillStation;



extern void FILL_init(SD_HandleTypeDef* sdHandler, FDCAN_HandleTypeDef* hfdcan);

extern void FILL_tick(uint32_t tick);

/* Queue a valve command for transmission over FDCAN1. */
extern void FILL_sendValveCmd(uint8_t valve, uint8_t cmd);