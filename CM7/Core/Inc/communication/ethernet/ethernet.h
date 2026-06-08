#ifndef __TESTIP_H
#define __TESTIP_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_eth.h"

/* Public macros -------------------------------------------------------------*/
#define MAKE_IPV4_ADDR(b1, b2, b3, b4) ((uint32_t)(b4) | ((uint32_t)(b3) << 8) | ((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 24))

/* Public defines ------------------------------------------------------------*/
#define RX_BUF_SIZE 1536
#define RX_BUF_CNT  4

/* Structs -------------------------------------------------------------------*/
typedef enum {
	BUF_FREE,
	BUF_OWNED_CPU,
	BUF_OWNED_DMA
} BufStatus;

typedef struct {
	uint8_t mac[6];
	uint32_t ip;
	uint16_t port;
} NetAddr;

typedef struct __attribute__((packed)) {
	uint8_t dst[6];		// MAC destination
	uint8_t src[6];		// MAC source
	uint16_t ethertype;
} ETH_Header;

typedef struct __attribute__((packed)) {
	// BEGIN FIELD ORDER REVERSAL (LITTLE ENDIAN BITFIELD PACKING)
	uint8_t ihl: 4;					// Internet Header Length
	uint8_t version: 4;				// Version (4)
	// END FIELD ORDER REVERSAL (LITTLE ENDIAN BITFIELD PACKING)

	// BEGIN FIELD ORDER REVERSAL (LITTLE ENDIAN BITFIELD PACKING)
	uint8_t ecn: 2;					// Explicit Congestion Notification
	uint8_t dscp: 6;				// Differentiated Services Code Point
	// END FIELD ORDER REVERSAL (LITTLE ENDIAN BITFIELD PACKING)

	uint16_t len;					// Total Length
	uint16_t id;					// Identification

	// Contains the fields listed below
	// (3 bits)		Flags (R: Reserved | DF: Don't Fragment | MF: More Fragments)
	// (13 bits)	Fragment Offset
	#define IPV4_DF_FLAG 		0x4000
	#define IPV4_MF_FLAG 		0x2000
	#define IPV4_OFFSET_MASK 	0x1FFF
	uint16_t frag;

	uint8_t ttl;					// Time to live
	uint8_t protocol;
	uint16_t checksum;				// Header Checksum
	uint32_t src;					// Source Address
	uint32_t dst;					// Destination Address

	// Options field not included as its length is variable (0-320 bits)
} IPV4_Header;

typedef struct __attribute__((packed)) {
	uint8_t type;
	uint8_t code;
	uint16_t checksum;
} ICMP_Header;

typedef struct __attribute__((packed)) {
	uint8_t type;
	uint8_t code;
	uint16_t checksum;
	uint16_t id;		// Identifier
	uint16_t seq;		// Sequence Number
} ICMP_Echo_Header;

typedef struct __attribute__((packed)) {
	uint16_t srcPort;
	uint16_t dstPort;
	uint16_t len;
	uint16_t checksum;
} UDP_Header;

typedef struct __attribute__((packed)) {
	uint16_t htype;		// Hardware type
	uint16_t ptype;		// Protocol type
	uint8_t hlen;		// Hardware Address Length
	uint8_t plen;		// Protocol Address Length
	uint16_t oper;		// Operation
	uint8_t sha[6];		// Sender Hardware Address
	uint32_t spa;		// Sender Protocol Address
	uint8_t tha[6];		// Target Hardware Address
	uint32_t tpa;		// Target Protocol Address
} ARP_Packet;

typedef enum {
	PING_STATE_IDLE,
	PING_STATE_PENDING
} PingState;

typedef enum {
	PING_RES_SUCCESS,
	PING_RES_TIMEOUT
} PingStatus;

typedef struct {
	PingState state;
	uint32_t targetIp;
	uint16_t id;
	uint16_t seq;
	uint32_t sentTick;
} PingControlBlock;

/* Public function prototypes  -----------------------------------------------*/
void TESTIP_Init();
void TESTIP_Process();
uint8_t* TESTIP_GetDataPtr();
void TESTIP_ProcessETHFrame(uint8_t *frame);
HAL_StatusTypeDef TESTIP_SendUDPPacket(NetAddr *netAddr, uint16_t len);
HAL_StatusTypeDef TESTIP_Ping(NetAddr *netAddr);

/* Callbacks -----------------------------------------------------------------*/
void TESTIP_UDP_RxCpltCallback(NetAddr *netAddr, uint8_t *payload, uint16_t len);
void TESTIP_PingCallback(uint32_t ip, PingStatus status, uint32_t rtt_ms);

#endif // __TESTIP_H
