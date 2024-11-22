#ifndef _MACHINE_MK6X_SDIO_
#define _MACHINE_MK6X_SDIO_

#include <stdint.h>
#include <sys/types.h>
#include <machine/sd_card_info.h>
#include "sd_card_info.h"

#define SD_MAX_INIT_RATE_KHZ 400
#define SDHC_PROCTL_DTW_4BIT 0x01

#define SDIOCONFIG_FLAG_SET(fl) (m_sdioConfig |= fl)
#define SDIOCONFIG_FLAG_UNSET(fl) (m_sdioConfig &= ~fl)
#define SDIOCONFIG_FLAG_GET(fl) (m_sdioConfig & fl)

#define SDIOCONFIG_USE_DMA_FLAG (1 << 0)
#define SDIOCONFIG_USE_DMA SDIOCONFIG_FLAG_GET(SDIOCONFIG_USE_DMA_FLAG)

#define ENABLE_TEENSY_SDIOMOD 1

void mk6x_sdio_isr(void);
int mk6x_sdio_init(int config);
int mk6x_sdio_erase(uint32_t firstSector, uint32_t lastSector);
uint8_t mk6x_sdio_errorCode();
uint32_t mk6x_sdio_errorData();
uint32_t mk6x_sdio_errorLine();
int mk6x_sdio_isBusy();
uint32_t mk6x_sdio_kHzSdClk();
int mk6x_sdio_readCID(cid_t *cid);
int mk6x_sdio_readCSD(csd_t *csd);
int mk6x_sdio_readData(char *dst);
int mk6x_sdio_readOCR(uint32_t *ocr);
int mk6x_sdio_readSector(uint32_t sector, char *dst);
int mk6x_sdio_readSectors(uint32_t sector, char *dst, size_t n);
int mk6x_sdio_readStart(uint32_t sector);
int mk6x_sdio_readStop();
uint32_t mk6x_sdio_sectorCount();
uint32_t mk6x_sdio_status();
int mk6x_sdio_stopTransmission(int blocking);
int mk6x_sdio_syncDevice();
uint8_t mk6x_sdio_cardtype();
int mk6x_sdio_writeData(char *src);
int mk6x_sdio_writeSector(uint32_t sector, char *src);
int mk6x_sdio_writeSectors(uint32_t sector, char *src, size_t n);
int mk6x_sdio_writeStart(uint32_t sector);
int mk6x_sdio_writeStop();

#endif /* _MACHINE_MK6X_SDIO_ */
