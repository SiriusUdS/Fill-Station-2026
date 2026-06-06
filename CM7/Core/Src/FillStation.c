#include "FillStation.h"
#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "string.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"
#include "CRC.h"
#include "dil/ipc_can.h"
#include "dil/can_types.h"

static volatile FillStation fill;

void safeHandler(UDPPacketHeader header, const uint8_t* payload){
}

void unsafeHandler(UDPPacketHeader header, const uint8_t* payload){
    
}

void igniteHandler(UDPPacketHeader header, const uint8_t* payload){
    
}

void testHandler(UDPPacketHeader header, const uint8_t* payload){
    
}

void errorHandler(UDPPacketHeader header, const uint8_t* payload){
    
}

void defaultHandler(UDPPacketHeader header, const uint8_t* payload){
    if(header.frame.payloadID == REQUEST_STATE){
        uint8_t state = payload[15];

        switch (fill.fillState)
        {
        case FILLING_STATION_STATE_SAFE:
            if (state != FILLING_STATION_STATE_TEST &&
                state != FILLING_STATION_STATE_UNSAFE)
                return;
            break;

        case FILLING_STATION_STATE_TEST:
            if (state != FILLING_STATION_STATE_SAFE)
                return;
            break;

        case FILLING_STATION_STATE_UNSAFE:
            if (state != FILLING_STATION_STATE_SAFE &&
                state != FILLING_STATION_STATE_IGNITE &&
                state != FILLING_STATION_STATE_ABORT)
                return;
            break;

        case FILLING_STATION_STATE_IGNITE:
            if (state != FILLING_STATION_STATE_SAFE &&
                state != FILLING_STATION_STATE_ABORT)
                return;
            break;

        case FILLING_STATION_STATE_ABORT:
            if (state != FILLING_STATION_STATE_SAFE)
                return;
            break;

        default:
            return;
        }

        fill.fillState = state;
    }
}

void messageHandler(UDPPacketHeader header, const uint8_t* payload){

    if(header.frame.deviceID != FILLING_STATION_BOARD_ID && header.frame.deviceID != BOARD_BROADCAST_ID){
        //CAN parser
        return;
    }else if(header.frame.deviceID == BOARD_BROADCAST_ID){
        defaultHandler(header, payload);
        return;
    }

    switch (fill.fillState) // COMMAND
    {
    case FILLING_STATION_STATE_SAFE:
        safeHandler(header, payload);
        break;
    case FILLING_STATION_STATE_UNSAFE:
        unsafeHandler(header, payload);
        break;
    case FILLING_STATION_STATE_IGNITE:
        igniteHandler(header, payload);
        break;
    case FILLING_STATION_STATE_ERROR:
        errorHandler(header, payload);
        break;
    case FILLING_STATION_STATE_TEST:
        testHandler(header, payload);
        break;
    default:
        break;
    }
    
    defaultHandler(header, payload);//default command
}

void TESTIP_UDP_RxCpltCallback(NetAddr *netAddr, uint8_t *payload, uint16_t len) {
  //CMD from the GS
  UDPPacketHeader header;
  memcpy(header.bytes, payload, sizeof(header));
  fill.lastEthernetMessageRx_MS = HAL_GetTick();
  messageHandler(header, payload);
}


static void messageTick(){
    
    if(fill.fillState == FILLING_STATION_STATE_UNSAFE || fill.fillState == FILLING_STATION_STATE_IGNITE){
        if((fill.currentTick - fill.lastEthernetMessageRx_MS) >= RX_WATCHDOG){
           fill.fillState = FILLING_STATION_STATE_ABORT; 
        }
    }
    //HEARTBEAT
    if(fill.currentTick- fill.lastEthernetMessageTx_MS < 1) return;
    uint8_t* buffer = TESTIP_GetDataPtr();
    UDPPacketHeader header;
    header.frame.deviceID = FILLING_STATION_BOARD_ID;
    header.frame.deviceState = fill.fillState;
    header.frame.deviceTS_MS = fill.currentTick;
    header.frame.payloadID = GET_SYSTEM;
    header.frame.payloadLenght = 4;
    uint8_t values[] = {0xDE,0xAD,0xBE,0xEF};
    uint32_t crc = CRC32_Calculate(values, 4);
    // Write header then CRC directly into buffer — no intermediate sizedBuf needed
    memcpy(buffer, header.bytes, sizeof(UDPPacketHeader));
    memcpy(buffer + sizeof(UDPPacketHeader), values, sizeof(uint32_t));
    memcpy(buffer+sizeof(UDPPacketHeader)+4, &crc, sizeof(uint32_t));
    TESTIP_SendUDPPacket(&(fill.gsAddr), sizeof(UDPPacketHeader)+8);
    fill.lastEthernetMessageTx_MS = fill.currentTick;
}

static void valveTick(){

}

/* Drain CAN frames received from the bus (published by the M4) and keep the
   latest one available to the controller. */
static void canTick(){
    uint32_t id;
    uint8_t  d[8];
    while (IpcCan_Receive(&id, d)) {
        fill.canRxCount++;
        fill.canLastRxId = id;
        for (uint32_t i = 0u; i < 8u; i++) {
            fill.canLastRxData[i] = d[i];
        }
        /* TODO: dispatch to the state machine, e.g. decode CAN_ID_STATUS_VALVE. */
    }
}

void FILL_sendValveCmd(uint8_t valve, uint8_t cmd){
    CANHeader h;
    h.code = 0;
    h.frame.senderID    = CAN_NODE_FCU;
    h.frame.targetID    = CAN_NODE_ECU;
    h.frame.deviceState = cmd;
    h.frame.messageID   = CAN_ID_CMD_VALVE;
    h.frame.errorCtrl   = 0;
    h.frame.errorCode   = 0;
    h.frame.reserved    = 0;

    /* Payload mirrors ValveCmdPayload: timeStamp_ms (4 bytes) + valveIndex. */
    uint32_t ts = fill.currentTick;
    uint8_t d[8] = {0};
    d[0] = (uint8_t)(ts & 0xFFu);
    d[1] = (uint8_t)((ts >> 8) & 0xFFu);
    d[2] = (uint8_t)((ts >> 16) & 0xFFu);
    d[3] = (uint8_t)((ts >> 24) & 0xFFu);
    d[4] = valve;

    IpcCan_Send(h.code, d);
}


static void STATE_test(){

}


static void STATE_safe(){

}

static void STATE_unsafe(){

}

static void STATE_ignite(){

}

static void STATE_abort(){

}

static void STATE_error(){

}

void FILL_init(SD_HandleTypeDef* sdHandler)
{
    fill.fillState = FILLING_STATION_STATE_INIT;
    fill.lastEthernetMessageRx_MS = 0;
    fill.lastEthernetMessageTx_MS = 0;
    fill.gsAddr.mac[0] = 0x00;
    fill.gsAddr.mac[1] = 0xe0;
    fill.gsAddr.mac[2] = 0x4c;
    fill.gsAddr.mac[3] = 0x33;
    fill.gsAddr.mac[4] = 0x0f;
    fill.gsAddr.mac[5] = 0x98;
    fill.gsAddr.ip = MAKE_IPV4_ADDR(192, 168, 0, 111);
    fill.gsAddr.port = 7520;

    TESTIP_Init();

    SDCARD_init(&fill.sd, sdHandler);

    uint8_t test[] = "ALLO!!!";
    SDCARD_write(&fill.sd, test, sizeof(test));
}

void FILL_tick(uint32_t tick)
{
    fill.currentTick = tick;
    canTick();
    messageTick();
    valveTick();
    
    switch (fill.fillState)
    {
        case FILLING_STATION_STATE_INIT:
            fill.fillState = FILLING_STATION_STATE_SAFE;
            TESTIP_Ping(&(fill.gsAddr));
        case FILLING_STATION_STATE_TEST:
            STATE_test();
            break;
        case FILLING_STATION_STATE_SAFE:
            STATE_safe();
            break;
        case FILLING_STATION_STATE_UNSAFE:
            STATE_unsafe();
            break;
        case FILLING_STATION_STATE_IGNITE:
            STATE_ignite();
            break;
        case FILLING_STATION_STATE_ERROR:
            STATE_error();
            break;
        case FILLING_STATION_STATE_ABORT:
            STATE_abort();
            break;
        default:
            break;
    }
}
