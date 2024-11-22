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

#define MAX_BLKCNT 0XFFFF

#define FIFO_WML 16
#define CMD8_RETRIES 3
#define BUSY_TIMEOUT_MS 1000

#define SDHC_IRQSTATEN_MASK                                            \
    (SDHC_IRQSTATEN_DMAESEN | SDHC_IRQSTATEN_AC12ESEN |                \
     SDHC_IRQSTATEN_DEBESEN | SDHC_IRQSTATEN_DCESEN |                  \
     SDHC_IRQSTATEN_DTOESEN | SDHC_IRQSTATEN_CIESEN |                  \
     SDHC_IRQSTATEN_CEBESEN | SDHC_IRQSTATEN_CCESEN |                  \
     SDHC_IRQSTATEN_CTOESEN | SDHC_IRQSTATEN_DINTSEN |                 \
     SDHC_IRQSTATEN_TCSEN | SDHC_IRQSTATEN_CCSEN)
#define SDHC_IRQSTAT_CMD_ERROR                                         \
    (SDHC_IRQSTAT_CIE | SDHC_IRQSTAT_CEBE | SDHC_IRQSTAT_CCE |         \
     SDHC_IRQSTAT_CTOE)
#define SDHC_IRQSTAT_DATA_ERROR                                        \
    (SDHC_IRQSTAT_AC12E | SDHC_IRQSTAT_DEBE | SDHC_IRQSTAT_DCE |       \
     SDHC_IRQSTAT_DTOE)
#define SDHC_IRQSTAT_ERROR                                             \
    (SDHC_IRQSTAT_DMAE | SDHC_IRQSTAT_CMD_ERROR |                      \
     SDHC_IRQSTAT_DATA_ERROR)
#define SDHC_IRQSIGEN_MASK                                             \
    (SDHC_IRQSIGEN_DMAEIEN | SDHC_IRQSIGEN_AC12EIEN |                  \
     SDHC_IRQSIGEN_DEBEIEN | SDHC_IRQSIGEN_DCEIEN |                    \
     SDHC_IRQSIGEN_DTOEIEN | SDHC_IRQSIGEN_CIEIEN |                    \
     SDHC_IRQSIGEN_CEBEIEN | SDHC_IRQSIGEN_CCEIEN |                    \
     SDHC_IRQSIGEN_CTOEIEN | SDHC_IRQSIGEN_TCIEN)
#define CMD_RESP_NONE  (SDHC_XFERTYP_RSPTYP(0))
#define CMD_RESP_R1                                                    \
    (SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(2))
#define CMD_RESP_R1b                                                   \
    (SDHC_XFERTYP_CICEN | SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(3))
#define CMD_RESP_R2 (SDHC_XFERTYP_CCCEN | SDHC_XFERTYP_RSPTYP(1))
#define CMD_RESP_R3 (SDHC_XFERTYP_RSPTYP(2))
#define CMD_RESP_R6 (CMD_RESP_R1)
#define CMD_RESP_R7 (CMD_RESP_R1)


#define DATA_READ (SDHC_XFERTYP_DTDSEL | SDHC_XFERTYP_DPSEL)
#define DATA_READ_DMA (DATA_READ | SDHC_XFERTYP_DMAEN)
#define DATA_READ_MULTI_DMA                                            \
    (DATA_READ_DMA | SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_AC12EN |       \
     SDHC_XFERTYP_BCEN)
#define DATA_READ_MULTI_PGM                                            \
    (DATA_READ | SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_BCEN)
#define DATA_WRITE_DMA (SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DMAEN)
#define DATA_WRITE_MULTI_DMA                                           \
    (DATA_WRITE_DMA | SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_AC12EN |      \
     SDHC_XFERTYP_BCEN)
#define DATA_WRITE_MULTI_PGM                                           \
    (SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_BCEN)

#define ACMD6_XFERTYP (SDHC_XFERTYP_CMDINX(ACMD6) | CMD_RESP_R1)
#define ACMD41_XFERTYP (SDHC_XFERTYP_CMDINX(ACMD41) | CMD_RESP_R3)
#define CMD0_XFERTYP (SDHC_XFERTYP_CMDINX(CMD0) | CMD_RESP_NONE)
#define CMD2_XFERTYP (SDHC_XFERTYP_CMDINX(CMD2) | CMD_RESP_R2)
#define CMD3_XFERTYP (SDHC_XFERTYP_CMDINX(CMD3) | CMD_RESP_R6)
#define CMD6_XFERTYP                                                   \
    (SDHC_XFERTYP_CMDINX(CMD6) | CMD_RESP_R1 | DATA_READ_DMA)
#define CMD7_XFERTYP (SDHC_XFERTYP_CMDINX(CMD7) | CMD_RESP_R1b)
#define CMD8_XFERTYP (SDHC_XFERTYP_CMDINX(CMD8) | CMD_RESP_R7)
#define CMD9_XFERTYP (SDHC_XFERTYP_CMDINX(CMD9) | CMD_RESP_R2)
#define CMD10_XFERTYP (SDHC_XFERTYP_CMDINX(CMD10) | CMD_RESP_R2)
#define CMD11_XFERTYP (SDHC_XFERTYP_CMDINX(CMD11) | CMD_RESP_R1)
#define CMD12_XFERTYP                                                  \
    (SDHC_XFERTYP_CMDINX(CMD12) | CMD_RESP_R1b | SDHC_XFERTYP_CMDTYP(3))
#define CMD13_XFERTYP (SDHC_XFERTYP_CMDINX(CMD13) | CMD_RESP_R1)
#define CMD17_DMA_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD17) | CMD_RESP_R1 | DATA_READ_DMA)
#define CMD18_DMA_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD18) | CMD_RESP_R1 | DATA_READ_MULTI_DMA)
#define CMD18_PGM_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD18) | CMD_RESP_R1 | DATA_READ_MULTI_PGM)
#define CMD24_DMA_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD24) | CMD_RESP_R1 | DATA_WRITE_DMA)
#define CMD25_DMA_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD25) | CMD_RESP_R1 | DATA_WRITE_MULTI_DMA)
#define CMD25_PGM_XFERTYP                                              \
    (SDHC_XFERTYP_CMDINX(CMD25) | CMD_RESP_R1 | DATA_WRITE_MULTI_PGM)
#define CMD32_XFERTYP (SDHC_XFERTYP_CMDINX(CMD32) | CMD_RESP_R1)
#define CMD33_XFERTYP (SDHC_XFERTYP_CMDINX(CMD33) | CMD_RESP_R1)
#define CMD38_XFERTYP (SDHC_XFERTYP_CMDINX(CMD38) | CMD_RESP_R1b)
#define CMD55_XFERTYP (SDHC_XFERTYP_CMDINX(CMD55) | CMD_RESP_R1)


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
