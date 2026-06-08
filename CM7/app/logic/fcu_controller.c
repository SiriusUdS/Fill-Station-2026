#include "fcu_controller.h"

static volatile FillStation fill;

/* ---- CAN protocol layer (runs on the M7, transported directly over FDCAN1
   by the CanBus_* shim) ----------------------------------------------------- */
static CanNodeId s_fcuNode = CAN_NODE_FCU;   /* PONG sender id for handler_ping */

/* FCU-side handler for valve-status frames returned by the Engine (ECU). */
static void fillValveStatusHandler(void *ctx, const CANHeader *header,
                                   const uint8_t *rxData)
{
    (void)ctx;
    CanValveIndex  valve;
    CanValveStatus status;
    uint32_t       ts;
    valveStatusPacketParse(header->code, rxData, &valve, &status, &ts);

    fill.canRxCount++;
    fill.canLastRxId = header->code;
    for (uint32_t i = 0u; i < 8u; i++) {
        fill.canLastRxData[i] = rxData[i];
    }
    /* TODO: feed `valve`/`status` into the fill state machine. */
}

static const CANHandlerEntry s_canHandlers[] = {
    { CAN_ID_COMM_PING,    handler_ping,           &s_fcuNode },
    { CAN_ID_STATUS_VALVE, fillValveStatusHandler, NULL       },
};

static CANControllerConfig s_canCfg = {
    .nodeID       = CAN_NODE_FCU,
    .handlers     = s_canHandlers,
    .handlerCount = (uint8_t)(sizeof(s_canHandlers) / sizeof(s_canHandlers[0])),
};

static CANController s_can;

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

/* Drain every CAN frame received on FDCAN1 (queued by the RX FIFO0 ISR) and
   dispatch it through the controller (ping -> PONG, valve-status -> fillValveStatusHandler). */
static void canTick(){
    while (CANController_Process(&s_can));
}

void FILL_sendValveCmd(uint8_t valve, uint8_t cmd){
    ValveCmdPacket pkt;
    valveCmdPacketMake(CAN_NODE_FCU, CAN_NODE_ECU,
                       (CanValveIndex)valve, (CanValveCmd)cmd, &pkt);
    CanBus_Send(pkt.header.code, pkt.payload.data);
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

void FILL_init(SD_HandleTypeDef* sdHandler, FDCAN_HandleTypeDef* hfdcan)
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

    /* Start FDCAN1: install the FCU RX filter, start the peripheral and enable
       the RX FIFO0 interrupt. The peripheral itself is created in MX_FDCAN1_Init(). */
    CanBus_Init(hfdcan, CAN_NODE_FCU);
    CANController_Init(&s_can, &s_canCfg);

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
