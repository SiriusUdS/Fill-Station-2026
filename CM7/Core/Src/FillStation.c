#include "FillStation.h"
#include "sirius-headers-common/Ethernet/UDPFrame.h"
#include "sirius-headers-common/Telecommunication/PacketHeaderVariable.h"
#include "string.h"
#include "sirius-headers-common/Telecommunication/BoardCommandV2.h"

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
    header.frame.payloadLenght = 0;
    memcpy(buffer, header.bytes, sizeof(UDPPacketHeader));
    TESTIP_SendUDPPacket(&(fill.gsAddr), sizeof(UDPPacketHeader));
    fill.lastEthernetMessageTx_MS = fill.currentTick;
}

static void valveTick(){

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

void FILL_init()
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
}

void FILL_tick(uint32_t tick)
{
    fill.currentTick = tick;
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
