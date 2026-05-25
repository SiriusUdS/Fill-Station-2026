#pragma once
#include "stm32h7xx_hal.h"
#include "fatfs.h"

#include <stdint.h>

#define SD_CARD_INIT 0x0
#define SD_CARD_IDLE 0x1
#define SD_CARD_BSY 0x2
#define SD_CARD_ERROR 0x3
#define SD_CARD_NOP 0x4



#define SD_CARD_ERNO_MOUNT_FAIL 0x01
#define SD_CARD_ERNO_FILE_OPEN_FAIL 0x02
#define SD_CARD_ERNO_FILE_WRITE_FAIL 0x03

typedef union {
    struct {
        uint16_t mounted:1;
        uint16_t init:1;
        uint16_t fileOpen:1;
        uint16_t RDY:1;
        uint16_t detected:1;
        uint16_t ernoEnable:1;
        uint16_t erno: 8;
        uint16_t reserved:2;
    }map;

    uint16_t code;
} SDCardStatus;


typedef struct {
    SD_HandleTypeDef* externalHandler;
    SDCardStatus status;
    uint8_t state;

    FATFS fs;

} SDCard;


extern void SDCARD_init(SDCard* instance, SD_HandleTypeDef* handler);

extern void SDCARD_write(SDCard* instance, const uint8_t* buffer, size_t length);