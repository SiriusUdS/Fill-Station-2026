#include "SDCard.h"


static void sendErno(SDCardStatus* stat, uint8_t code){
    stat->map.ernoEnable = 1;
    stat->map.erno = code;
}

static void unmount(SDCard* instance){
    if(!instance->status.map.mounted) return;
    
    f_mount(NULL, "0:/", 0);
    instance->status.map.mounted = 0;
}

void SDCARD_init(SDCard* instance, SD_HandleTypeDef* handler){ 
    instance->externalHandler = handler;
    FRESULT fr;
    fr = f_mount(&instance->fs, SDPath, 1);
    if(fr != FR_OK){
        instance->status.map.mounted = 0;
        instance->status.map.init = 0;
        sendErno(&instance->status, SD_CARD_ERNO_MOUNT_FAIL);
        return;
    }

    instance->status.map.mounted = 1;
    instance->state = SD_CARD_IDLE;
    
}

void SDCARD_write(SDCard *instance, const uint8_t *buffer, size_t length)
{
    if(instance->state != SD_CARD_IDLE) return;

    instance->state = SD_CARD_BSY;
    FIL file;
    FRESULT fr;
    fr = f_open(&file, "0:/runtime.bin", FA_WRITE | FA_CREATE_ALWAYS);

    if(fr != FR_OK){
        sendErno(&instance->status, SD_CARD_ERNO_FILE_OPEN_FAIL);
        return;
    }
    size_t written = 0;
    f_write(&file, buffer, length, &written);

    if(written != length){
        instance->state = SD_CARD_ERROR;
        sendErno(&instance->status, SD_CARD_ERNO_FILE_WRITE_FAIL);
        return;
    }

    f_close(&file);
    instance->state = SD_CARD_IDLE;
    unmount(instance);
}
