/**
 *
 * Copyright (c) 2011-2021 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>

#include <sys/param.h>
#include <sys/uio.h>
#include <sys/kconfig.h>

#include <machine/intr.h>
#include <machine/sdio.h>
#include <machine/mk6x_sdio.h>
#include <machine/sd_card_info.h>
#include <machine/kinetis.h>
#include <machine/intr.h>
#include <machine/systick.h>

/* in other words, this only works on Teensy 3.5/3.6 and 4.x. */
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) ||                \
    defined(__IMXRT1062__)

static int cardCommand(uint32_t xfertyp, uint32_t arg);
static void enableGPIO(int enable);
static void enable_dma_isr();
static void initSDHC();
static int isBusyCMD13();
static int isBusyCommandComplete();
static int isBusyCommandInhibit();
static int readReg16(uint32_t xfertyp, void *data);
static void setSdclk(uint32_t kHzMax);
static int yieldTimeout(int (*fcn)());
static int waitDmaStatus();
static int waitTimeout(int (*fcn)());

static int (*m_busyFcn)() = 0;
static int m_initDone     = 0;
static int m_version2;
static int m_highCapacity;
static int m_transferActive = 0;
static int m_sdioConfig     = 0;
uint8_t m_errorCode         = SD_CARD_ERROR_INIT_NOT_CALLED;
uint32_t m_errorLine        = 0;
static uint32_t m_rca;
static volatile int m_dmaBusy = 0;
static volatile uint32_t m_irqstat;
static uint32_t m_sdClkKhz = 0;
static uint32_t m_ocr;
static cid_t m_cid;
static csd_t m_csd;

static const uint8_t IDLE_STATE  = 0;
static const uint8_t READ_STATE  = 1;
static const uint8_t WRITE_STATE = 2;

uint8_t m_curState               = IDLE_STATE;
uint32_t m_curSector;

#define ALIGNMENT_TARGET 7

#define DBG_TRACE        printf("TRACE: " __LINE__ "\n");
#define USE_DEBUG_MODE   0
#if USE_DEBUG_MODE
#define DBG_IRQSTAT()                                                  \
    if (SDHC_IRQSTAT) {                                                \
        printf(__LINE__);                                              \
        printf(" IRQSTAT 0x%x\n", SDHC_IRQSTAT);                       \
    }
static void printRegs(uint32_t line) {
    uint32_t blkattr = SDHC_BLKATTR;
    uint32_t xfertyp = SDHC_XFERTYP;
    uint32_t prsstat = SDHC_PRSSTAT;
    uint32_t proctl  = SDHC_PROCTL;
    uint32_t irqstat = SDHC_IRQSTAT;

#if 0  
  Serial.print("\nLINE: ");
  Serial.println(line);
  Serial.print("BLKATTR ");
  Serial.println(blkattr, HEX);
  Serial.print("XFERTYP ");
  Serial.print(xfertyp, HEX);
  Serial.print(" CMD");
  Serial.print(xfertyp >> 24);
  Serial.print(" TYP");
  Serial.print((xfertyp >> 2) & 3);
  if (xfertyp & SDHC_XFERTYP_DPSEL) {Serial.print(" DPSEL");}
  Serial.println();
  Serial.print("PRSSTAT ");
  Serial.print(prsstat, HEX);
  if (prsstat & SDHC_PRSSTAT_BREN) {Serial.print(" BREN");}
  if (prsstat & SDHC_PRSSTAT_BWEN) {Serial.print(" BWEN");}
  if (prsstat & SDHC_PRSSTAT_RTA) {Serial.print(" RTA");}
  if (prsstat & SDHC_PRSSTAT_WTA) {Serial.print(" WTA");}
  if (prsstat & SDHC_PRSSTAT_SDOFF) {Serial.print(" SDOFF");}
  if (prsstat & SDHC_PRSSTAT_PEROFF) {Serial.print(" PEROFF");}
  if (prsstat & SDHC_PRSSTAT_HCKOFF) {Serial.print(" HCKOFF");}
  if (prsstat & SDHC_PRSSTAT_IPGOFF) {Serial.print(" IPGOFF");}
  if (prsstat & SDHC_PRSSTAT_SDSTB) {Serial.print(" SDSTB");}
  if (prsstat & SDHC_PRSSTAT_DLA) {Serial.print(" DLA");}
  if (prsstat & SDHC_PRSSTAT_CDIHB) {Serial.print(" CDIHB");}
  if (prsstat & SDHC_PRSSTAT_CIHB) {Serial.print(" CIHB");}
  Serial.println();
  Serial.print("PROCTL ");
  Serial.print(proctl, HEX);
  if (proctl & SDHC_PROCTL_SABGREQ) Serial.print(" SABGREQ");
  Serial.print(" EMODE");
  Serial.print((proctl >>4) & 3);
  Serial.print(" DWT");
  Serial.print((proctl >>1) & 3);
  Serial.println();
  Serial.print("IRQSTAT ");
  Serial.print(irqstat, HEX);
  if (irqstat & SDHC_IRQSTAT_BGE) {Serial.print(" BGE");}
  if (irqstat & SDHC_IRQSTAT_TC) {Serial.print(" TC");}
  if (irqstat & SDHC_IRQSTAT_CC) {Serial.print(" CC");}
  Serial.print("\nm_irqstat ");
  Serial.println(m_irqstat, HEX);
#endif
}
#else // USE_DEBUG_MODE
#define DBG_IRQSTAT()
#endif // USE_DEBUG_MODE

// Error function and macro.
#define sdError(code) setSdErrorCode(code, __LINE__)
inline int setSdErrorCode(uint8_t code, uint32_t line) {
    m_errorCode = code;
    m_errorLine = line;
#if USE_DEBUG_MODE
    printRegs(line);
#endif // USE_DEBUG_MODE
    return 0;
}

// ISR
void sdhc_isr(void) {
    SDHC_IRQSIGEN = 0;
    m_irqstat     = SDHC_IRQSTAT;
    SDHC_IRQSTAT  = m_irqstat;
#if defined(__IMXRT1062__)
    SDHC_MIX_CTRL &= ~(SDHC_MIX_CTRL_AC23EN | SDHC_MIX_CTRL_DMAEN);
#endif
    m_dmaBusy = 0;
}

// GPIO and clock functions.
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

static void enableGPIO(int enable) {
    const uint32_t PORT_CLK      = PORT_PCR_MUX(4) | PORT_PCR_DSE;
    const uint32_t PORT_CMD_DATA = PORT_CLK | PORT_PCR_PE | PORT_PCR_PS;
    const uint32_t PORT_PUP =
        PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;

    PORTE_PCR0 = enable ? PORT_CMD_DATA : PORT_PUP; // SDHC_D1
    PORTE_PCR1 = enable ? PORT_CMD_DATA : PORT_PUP; // SDHC_D0
    PORTE_PCR2 = enable ? PORT_CLK : PORT_PUP;      // SDHC_CLK
    PORTE_PCR3 = enable ? PORT_CMD_DATA : PORT_PUP; // SDHC_CMD
    PORTE_PCR4 = enable ? PORT_CMD_DATA : PORT_PUP; // SDHC_D3
    PORTE_PCR5 = enable ? PORT_CMD_DATA : PORT_PUP; // SDHC_D2
}

static void initClock() {
#ifdef HAS_KINETIS_MPU
    // Allow SDHC Bus Master access.
    MPU_RGDAAC0 |= 0x0C000000;
#endif // HAS_KINETIS_MPU
    // Enable SDHC clock.
    SIM_SCGC3 |= SIM_SCGC3_SDHC;
}
static uint32_t baseClock() { return F_CPU; }

#elif defined(__IMXRT1062__)

static void gpioMux(uint8_t mode) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04 = mode; // DAT2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = mode; // DAT3
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = mode; // CMD
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = mode; // CLK
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = mode; // DAT0
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = mode; // DAT1
}

// add speed strength args?
static void enableGPIO(int enable) {
    const uint32_t CLOCK_MASK = IOMUXC_SW_PAD_CTL_PAD_PKE |
#if defined(__IMXRT1062__)
                                IOMUXC_SW_PAD_CTL_PAD_DSE(7) |
#else  // defined(ARDUINO_TEENSY41)
                                IOMUXC_SW_PAD_CTL_PAD_DSE(4) | ///// WHG
#endif // defined(ARDUINO_TEENSY41)
                                IOMUXC_SW_PAD_CTL_PAD_SPEED(2);

    const uint32_t DATA_MASK = CLOCK_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE |
                               IOMUXC_SW_PAD_CTL_PAD_PUS(1);
    if (enable) {
        gpioMux(0);
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_04 = DATA_MASK;  // DAT2
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_05 = DATA_MASK;  // DAT3
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_00 = DATA_MASK;  // CMD
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_01 = CLOCK_MASK; // CLK
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_02 = DATA_MASK;  // DAT0
        IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_03 = DATA_MASK;  // DAT1
    } else {
        gpioMux(5);
    }
}

static void initClock() {
    /* set PDF_528 PLL2PFD0 */
    CCM_ANALOG_PFD_528 |= (1 << 7);
    CCM_ANALOG_PFD_528 &= ~(0x3F << 0);
    CCM_ANALOG_PFD_528 |= ((24) & 0x3F << 0); // 12 - 35
    CCM_ANALOG_PFD_528 &= ~(1 << 7);

    /* Enable USDHC clock. */
    CCM_CCGR6 |= CCM_CCGR6_USDHC1(CCM_CCGR_ON);
    CCM_CSCDR1 &= ~(CCM_CSCDR1_USDHC1_CLK_PODF_MASK);
    CCM_CSCMR1 |= CCM_CSCMR1_USDHC1_CLK_SEL; // PLL2PFD0
    //  CCM_CSCDR1 |= CCM_CSCDR1_USDHC1_CLK_PODF((7)); / &0x7  WHG
    CCM_CSCDR1 |= CCM_CSCDR1_USDHC1_CLK_PODF((1));
}

static uint32_t baseClock() {
    uint32_t divider = ((CCM_CSCDR1 >> 11) & 0x7) + 1;
    return (528000000U * 3) / ((CCM_ANALOG_PFD_528 & 0x3F) / 6) /
           divider;
}
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)

// Static functions.
static int cardAcmd(uint32_t rca, uint32_t xfertyp, uint32_t arg) {
    return cardCommand(CMD55_XFERTYP, rca) && cardCommand(xfertyp, arg);
}

static int cardCommand(uint32_t xfertyp, uint32_t arg) {
    DBG_IRQSTAT();
    if (waitTimeout(isBusyCommandInhibit)) {
        return 0; // Caller will set errorCode.
    }
    SDHC_CMDARG = arg;
#if defined(__IMXRT1062__)
    // Set MIX_CTRL if data transfer.
    if (xfertyp & SDHC_XFERTYP_DPSEL) {
        SDHC_MIX_CTRL &= ~SDHC_MIX_CTRL_MASK;
        SDHC_MIX_CTRL |= xfertyp & SDHC_MIX_CTRL_MASK;
    }
    xfertyp &= ~SDHC_MIX_CTRL_MASK;
#endif // defined(__IMXRT1062__)
    SDHC_XFERTYP = xfertyp;
    if (waitTimeout(isBusyCommandComplete)) {
        return 0; // Caller will set errorCode.
    }
    m_irqstat    = SDHC_IRQSTAT;
    SDHC_IRQSTAT = m_irqstat;

    return (m_irqstat & SDHC_IRQSTAT_CC) &&
           !(m_irqstat & SDHC_IRQSTAT_CMD_ERROR);
}

static int cardCMD6(uint32_t arg, uint8_t *status) {
    // CMD6 returns 64 bytes.
    if (waitTimeout(isBusyCMD13)) {
        return sdError(SD_CARD_ERROR_CMD13);
    }
    enable_dma_isr();
    SDHC_DSADDR   = (uint32_t) status;
    SDHC_BLKATTR  = SDHC_BLKATTR_BLKCNT(1) | SDHC_BLKATTR_BLKSIZE(64);
    SDHC_IRQSIGEN = SDHC_IRQSIGEN_MASK;
    if (!cardCommand(CMD6_XFERTYP, arg)) {
        return sdError(SD_CARD_ERROR_CMD6);
    }
    if (!waitDmaStatus()) {
        return sdError(SD_CARD_ERROR_DMA);
    }
    return 1;
}

static void enable_dma_isr() {
    m_dmaBusy = 1;
    m_irqstat = 0;
}

static void initSDHC() {
    initClock();

    // Disable GPIO clock.
    enableGPIO(0);

#if defined(__IMXRT1062__)
    SDHC_MIX_CTRL |= 0x80000000;
#endif //  (__IMXRT1062__)

    // Reset SDHC. Use default Water Mark Level of 16.
    SDHC_SYSCTL |= SDHC_SYSCTL_RSTA | SDHC_SYSCTL_SDCLKFS(0x80);

    while (SDHC_SYSCTL & SDHC_SYSCTL_RSTA) {
    }

    // Set initial SCK rate.
    setSdclk(SD_MAX_INIT_RATE_KHZ);

    enableGPIO(1);

    // Enable desired IRQSTAT bits.
    SDHC_IRQSTATEN = SDHC_IRQSTATEN_MASK;

    /* arm_isr_attach(IRQ_SDHC, mk6x_sdio_isr); */
    arm_set_irq_prio(IRQ_SDHC, SPL_BIO);
    arm_enable_irq(IRQ_SDHC);

    // Send 80 clocks to card.
    SDHC_SYSCTL |= SDHC_SYSCTL_INITA;
    while (SDHC_SYSCTL & SDHC_SYSCTL_INITA) {
    }
}

inline static uint32_t statusCMD13() {
    return cardCommand(CMD13_XFERTYP, m_rca) ? SDHC_CMDRSP0 : 0;
}

inline static int isBusyCMD13() {
    return !(statusCMD13() & CARD_STATUS_READY_FOR_DATA);
}

inline static int isBusyCommandComplete() {
    return !(SDHC_IRQSTAT & (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_CMD_ERROR));
}

inline static int isBusyCommandInhibit() {
    return SDHC_PRSSTAT & SDHC_PRSSTAT_CIHB;
}

inline static int isBusyDat() { return SDHC_PRSSTAT & (1 << 24) ? 0 : 1; }

inline static int isBusyDMA() { return m_dmaBusy; }

inline static int isBusyFifoRead() {
    return !(SDHC_PRSSTAT & SDHC_PRSSTAT_BREN);
}

inline static int isBusyFifoWrite() {
    return !(SDHC_PRSSTAT & SDHC_PRSSTAT_BWEN);
}

inline static int isBusyTransferComplete() {
    return !(SDHC_IRQSTAT & (SDHC_IRQSTAT_TC | SDHC_IRQSTAT_ERROR));
}

static int rdWrSectors(uint32_t xfertyp, uint32_t sector, char *buf,
                       size_t n) {
    if ((3 & (uint32_t) buf) || n == 0) {
        return sdError(SD_CARD_ERROR_DMA);
    }
    if (yieldTimeout(isBusyCMD13)) {
        return sdError(SD_CARD_ERROR_CMD13);
    }
    enable_dma_isr();
    SDHC_DSADDR   = (uint32_t) buf;
    SDHC_BLKATTR  = SDHC_BLKATTR_BLKCNT(n) | SDHC_BLKATTR_BLKSIZE(512);
    SDHC_IRQSIGEN = SDHC_IRQSIGEN_MASK;
    if (!cardCommand(xfertyp, m_highCapacity ? sector : 512 * sector)) {
        return 0;
    }
    return waitDmaStatus();
}

// Read 16 byte CID or CSD register.
static int readReg16(uint32_t xfertyp, void *data) {
    uint8_t *d = (data);
    if (!cardCommand(xfertyp, m_rca)) {
        return 0; // Caller will set errorCode.
    }
    uint32_t sr[] = {SDHC_CMDRSP0, SDHC_CMDRSP1, SDHC_CMDRSP2,
                     SDHC_CMDRSP3};
    for (int i = 0; i < 15; i++) {
        d[14 - i] = sr[i / 4] >> 8 * (i % 4);
    }
    d[15] = 0;
    return 1;
}

static void setSdclk(uint32_t kHzMax) {
    const uint32_t DVS_LIMIT     = 0X10;
    const uint32_t SDCLKFS_LIMIT = 0X100;
    uint32_t dvs                 = 1;
    uint32_t sdclkfs             = 1;
    uint32_t maxSdclk            = 1000 * kHzMax;
    uint32_t base                = baseClock();

    while ((base / (sdclkfs * DVS_LIMIT) > maxSdclk) &&
           (sdclkfs < SDCLKFS_LIMIT)) {
        sdclkfs <<= 1;
    }
    while ((base / (sdclkfs * dvs) > maxSdclk) && (dvs < DVS_LIMIT)) {
        dvs++;
    }
    m_sdClkKhz = base / (1000 * sdclkfs * dvs);
    sdclkfs >>= 1;
    dvs--;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Disable SDHC clock.
    SDHC_SYSCTL &= ~SDHC_SYSCTL_SDCLKEN;
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)

    // Change dividers.
    uint32_t sysctl =
        SDHC_SYSCTL & ~(SDHC_SYSCTL_DTOCV_MASK | SDHC_SYSCTL_DVS_MASK |
                        SDHC_SYSCTL_SDCLKFS_MASK);

    SDHC_SYSCTL = sysctl | SDHC_SYSCTL_DTOCV(0x0E) |
                  SDHC_SYSCTL_DVS(dvs) | SDHC_SYSCTL_SDCLKFS(sdclkfs);

    // Wait until the SDHC clock is stable.
    while (!(SDHC_PRSSTAT & SDHC_PRSSTAT_SDSTB)) {
    }

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Enable the SDHC clock.
    SDHC_SYSCTL |= SDHC_SYSCTL_SDCLKEN;
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)
}

static int transferStop() {
    // This fix allows CDIHB to be cleared in Tennsy 3.x without a reset.
    SDHC_PROCTL &= ~SDHC_PROCTL_SABGREQ;
    if (!cardCommand(CMD12_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD12);
    }
    if (yieldTimeout(isBusyDat)) {
        return sdError(SD_CARD_ERROR_CMD13);
    }
    if (SDHC_PRSSTAT & SDHC_PRSSTAT_CDIHB) {
        // This should not happen after above fix.
        // Save registers before reset DAT lines.
        uint32_t irqsststen = SDHC_IRQSTATEN;
        uint32_t proctl     = SDHC_PROCTL & ~SDHC_PROCTL_SABGREQ;
        // Do reset to clear CDIHB.  Should be a better way!
        SDHC_SYSCTL |= SDHC_SYSCTL_RSTD;
        // Restore registers.
        SDHC_IRQSTATEN = irqsststen;
        SDHC_PROCTL    = proctl;
    }
    return 1;
}

// return 1 if timeout occurs.
static int yieldTimeout(int (*fcn)()) {
    m_busyFcn  = fcn;
    uint32_t m = systick_ms;
    while (fcn()) {
        if ((systick_ms - m) > BUSY_TIMEOUT_MS) {
            m_busyFcn = 0;
	    printf("sdio: timeout\n");
            return 1;
        }
        /* yield(); */
    }
    m_busyFcn = 0;
    return 0; // Caller will set errorCode.
}

static int waitDmaStatus() {
    if (yieldTimeout(isBusyDMA)) {
        return 0; // Caller will set errorCode.
    }
    return (m_irqstat & SDHC_IRQSTAT_TC) &&
           !(m_irqstat & SDHC_IRQSTAT_ERROR);
}

// return 1 if timeout occurs.
static int waitTimeout(int (*fcn)()) {
    uint32_t m = systick_ms;
    while (fcn()) {
        if ((systick_ms - m) > BUSY_TIMEOUT_MS) {
            return 1;
        }
    }
    return 0; // Caller will set errorCode.
}

static int waitTransferComplete() {
    if (!m_transferActive) {
        return 1;
    }
    int timeOut      = waitTimeout(isBusyTransferComplete);
    m_transferActive = 0;
    m_irqstat        = SDHC_IRQSTAT;
    SDHC_IRQSTAT     = m_irqstat;
    if (timeOut || (m_irqstat & SDHC_IRQSTAT_ERROR)) {
        return sdError(SD_CARD_ERROR_TRANSFER_COMPLETE);
    }
    return 1;
}

int mk6x_sdio_init(int config) {
    uint32_t kHzSdClk;
    uint32_t arg;

    m_sdioConfig   = config;

    m_curState     = IDLE_STATE;
    m_initDone     = 0;
    m_errorCode    = SD_CARD_ERROR_NONE;
    m_highCapacity = 0;
    m_version2     = 0;

    // initialize controller.
    initSDHC();
    if (!cardCommand(CMD0_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD0);
    }
    // Try several times for case of reset delay.
    for (uint32_t i = 0; i < CMD8_RETRIES; i++) {
        if (cardCommand(CMD8_XFERTYP, 0X1AA)) {
            if (SDHC_CMDRSP0 != 0X1AA) {
                return sdError(SD_CARD_ERROR_CMD8);
            }
            m_version2 = 1;
            break;
        }
    }
#if defined(__IMXRT1062__)
    // Old version 1 cards have trouble with Teensy 4.1 after CMD8.
    // For reasons unknown, SDIO stops working after the cards does
    // not reply.  Simply restarting and CMD0 is a crude workaround.
    if (!m_version2) {
        initSDHC();
        cardCommand(CMD0_XFERTYP, 0);
    }
#endif
    arg   = m_version2 ? 0X40300000 : 0x00300000;
    int m = systick_ms;
    do {
        if (!cardAcmd(0, ACMD41_XFERTYP, arg) ||
            ((systick_ms - m) > BUSY_TIMEOUT_MS)) {
            return sdError(SD_CARD_ERROR_ACMD41);
        }
    } while ((SDHC_CMDRSP0 & 0x80000000) == 0);
    m_ocr = SDHC_CMDRSP0;
    if (SDHC_CMDRSP0 & 0x40000000) {
        // Is high capacity.
        m_highCapacity = 1;
    }
    if (!cardCommand(CMD2_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD2);
    }
    if (!cardCommand(CMD3_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD3);
    }
    m_rca = SDHC_CMDRSP0 & 0xFFFF0000;

    if (!readReg16(CMD9_XFERTYP, &m_csd)) {
        return sdError(SD_CARD_ERROR_CMD9);
    }
    if (!readReg16(CMD10_XFERTYP, &m_cid)) {
        return sdError(SD_CARD_ERROR_CMD10);
    }
    if (!cardCommand(CMD7_XFERTYP, m_rca)) {
        return sdError(SD_CARD_ERROR_CMD7);
    }
    // Set card to bus width four.
    if (!cardAcmd(m_rca, ACMD6_XFERTYP, 2)) {
        return sdError(SD_CARD_ERROR_ACMD6);
    }
    // Set SDHC to bus width four.
    SDHC_PROCTL &= ~SDHC_PROCTL_DTW_MASK;
    SDHC_PROCTL |= SDHC_PROCTL_DTW(SDHC_PROCTL_DTW_4BIT);

    SDHC_WML = SDHC_WML_RDWML(FIFO_WML) | SDHC_WML_WRWML(FIFO_WML);

    // Determine if High Speed mode is supported and set frequency.
    // Check status[16] for error 0XF or status[16] for new mode 0X1.
    uint8_t status[64];
    // Thanks to tompilot :-)
    // https://forum.pjrc.com/threads/69460?p=331762&viewfull=1#post331762
    // Ask if highspeed mode supported. Returns true if SD card reacts to the command within timeout.
    int highSpeedModeAsk       = cardCMD6(0X00FFFFFF, status);
    // Check the SD-card's response. This bit must be set.
    int highspeedModeSupported = (2 & status[13]);
    // safely ask for a switch request and accept in this case if there is no response.
    kHzSdClk                   = 25000;
    if (highSpeedModeAsk && highspeedModeSupported) {
        uint8_t err_code_before     = m_errorCode;
        uint32_t m_errorLine_before = m_errorLine;
        // Switch to highspeed mode request. Returns true if SD card reacts within timeout.
        int switchRequestAsk        = cardCMD6(0X80FFFFF1, status);
        // Check the SD-card's response. This bit must be set.
        int switchRequestDone       = ((status[16] & 0XF) == 1);
        if (switchRequestAsk && switchRequestDone) {
            kHzSdClk = 50000;
        } else {
            // Maybe there are cards that say they support highspeed mode, but won't respond to
            // a switch request.  If it says that highspeed mode was supported, but the card did
            // not react to the switch request, we accept the timeout in this case and proceed
            // with non-highspeed mode.
            // We also need to revert to the error code from before, as cardCMD6(0X80FFFFF1, status)
            // will change the error code & line otherwise.
            m_errorCode = err_code_before;
            m_errorLine = m_errorLine_before;
        }
    }
    // Disable GPIO.
    enableGPIO(0);

    // Set the SDHC SCK frequency.
    setSdclk(kHzSdClk);

    // Enable GPIO.
    enableGPIO(1);
    m_initDone = 1;
    return 1;
}

int mk6x_sdio_erase(uint32_t firstSector, uint32_t lastSector) {
#if ENABLE_TEENSY_SDIO_MOD
    if (m_curState != IDLE_STATE && !mk6x_sdio_syncDevice()) {
        return 0;
    }
#endif // ENABLE_TEENSY_SDIO_MOD
    // check for single sector erase
    if (!m_csd.v1.erase_blk_en) {
        // erase size mask
        uint8_t m =
            (m_csd.v1.sector_size_high << 1) | m_csd.v1.sector_size_low;
        if ((firstSector & m) != 0 || ((lastSector + 1) & m) != 0) {
            // error card can't erase specified area
            return sdError(SD_CARD_ERROR_ERASE_SINGLE_SECTOR);
        }
    }
    if (!m_highCapacity) {
        firstSector <<= 9;
        lastSector <<= 9;
    }
    if (!cardCommand(CMD32_XFERTYP, firstSector)) {
        return sdError(SD_CARD_ERROR_CMD32);
    }
    if (!cardCommand(CMD33_XFERTYP, lastSector)) {
        return sdError(SD_CARD_ERROR_CMD33);
    }
    if (!cardCommand(CMD38_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD38);
    }
    if (waitTimeout(isBusyCMD13)) {
        return sdError(SD_CARD_ERROR_ERASE_TIMEOUT);
    }
    return 1;
}

uint8_t mk6x_sdio_errorCode() { return m_errorCode; }

uint32_t mk6x_sdio_errorData() { return m_irqstat; }

uint32_t mk6x_sdio_errorLine() { return m_errorLine; }

int mk6x_sdio_isBusy() {
    if (SDIOCONFIG_USE_DMA) {
        return m_busyFcn ? m_busyFcn() : m_initDone && isBusyCMD13();
    } else {
        if (m_transferActive) {
            if (isBusyTransferComplete()) {
                return 1;
            }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
            if ((SDHC_BLKATTR & 0XFFFF0000) != 0) {
                return 0;
            }
            m_transferActive = 0;
            mk6x_sdio_stopTransmission(0);
            return 1;
#else  // defined(__MK64FX512__) || defined(__MK66FX1M0__)
            return 0;
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)
        }
        // Use DAT0 low as busy.
        return SDHC_PRSSTAT & (1 << 24) ? 0 : 1;
    }
}

uint32_t mk6x_sdio_kHzSdClk() { return m_sdClkKhz; }

int mk6x_sdio_readCID(cid_t *cid) {
    memcpy(cid, &m_cid, 16);
    return 1;
}

int mk6x_sdio_readCSD(csd_t *csd) {
    memcpy(csd, &m_csd, 16);
    return 1;
}

int mk6x_sdio_readData(char *dst) {
    DBG_IRQSTAT();
    uint32_t *p32 = (uint32_t *) (dst);
    int s;

    if (!(SDHC_PRSSTAT & SDHC_PRSSTAT_RTA)) {
        SDHC_PROCTL &= ~SDHC_PROCTL_SABGREQ;
	s = arm_disable_interrupts();
        SDHC_PROCTL |= SDHC_PROCTL_CREQ;
        SDHC_PROCTL |= SDHC_PROCTL_SABGREQ;
	arm_restore_interrupts(s);
    }
    if (waitTimeout(isBusyFifoRead)) {
        return sdError(SD_CARD_ERROR_READ_FIFO);
    }
    for (uint32_t iw = 0; iw < 512 / (4 * FIFO_WML); iw++) {
        while (0 == (SDHC_PRSSTAT & SDHC_PRSSTAT_BREN)) {
        }
        for (uint32_t i = 0; i < FIFO_WML; i++) {
            p32[i] = SDHC_DATPORT;
        }
        p32 += FIFO_WML;
    }
    if (waitTimeout(isBusyTransferComplete)) {
        return sdError(SD_CARD_ERROR_READ_TIMEOUT);
    }
    m_irqstat    = SDHC_IRQSTAT;
    SDHC_IRQSTAT = m_irqstat;
    return (m_irqstat & SDHC_IRQSTAT_TC) &&
           !(m_irqstat & SDHC_IRQSTAT_ERROR);
}

int mk6x_sdio_readOCR(uint32_t *ocr) {
    *ocr = m_ocr;
    return 1;
}

int mk6x_sdio_readSector(uint32_t sector, char *dst) {
    if (SDIOCONFIG_USE_DMA) {
        char aligned[512];

        char *ptr = (uint32_t) dst & ALIGNMENT_TARGET ? aligned : dst;

        if (!rdWrSectors(CMD17_DMA_XFERTYP, sector, ptr, 1)) {
            return sdError(SD_CARD_ERROR_CMD17);
        }
        if (ptr != dst) {
            memcpy(dst, aligned, 512);
        }
    } else {
        if (!waitTransferComplete()) {
            return 0;
        }
        if (m_curState != READ_STATE || sector != m_curSector) {
            if (!mk6x_sdio_syncDevice()) {
                return 0;
            }
            if (!mk6x_sdio_readStart(sector)) {
                return 0;
            }
            m_curSector = sector;
            m_curState  = READ_STATE;
        }
        if (!mk6x_sdio_readData(dst)) {
            return 0;
        }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        if ((SDHC_BLKATTR & 0XFFFF0000) == 0) {
            if (!mk6x_sdio_syncDevice()) {
                return 0;
            }
        }
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)
        m_curSector++;
    }
    return 1;
}

int mk6x_sdio_readSectors(uint32_t sector, char *dst, size_t n) {
    if (SDIOCONFIG_USE_DMA) {
        if ((uint32_t) dst & 3) {
            for (size_t i = 0; i < n; i++, sector++, dst += 512) {
                if (!mk6x_sdio_readSector(sector, dst)) {
                    return 0; // readSector will set errorCode.
                }
            }
            return 1;
        }
        if (!rdWrSectors(CMD18_DMA_XFERTYP, sector, dst, n)) {
            return sdError(SD_CARD_ERROR_CMD18);
        }
    } else {
        for (size_t i = 0; i < n; i++) {
            if (!mk6x_sdio_readSector(sector + i, dst + i * 512UL)) {
                return 0;
            }
        }
    }
    return 1;
}

// SDHC will do Auto CMD12 after count sectors.
int mk6x_sdio_readStart(uint32_t sector) {
    DBG_IRQSTAT();
    if (yieldTimeout(isBusyCMD13)) {
        return sdError(SD_CARD_ERROR_CMD13);
    }
    SDHC_PROCTL |= SDHC_PROCTL_SABGREQ;
#if defined(__IMXRT1062__)
    // Infinite transfer.
    SDHC_BLKATTR = SDHC_BLKATTR_BLKSIZE(512);
#else  // defined(__IMXRT1062__)
    // Errata - can't do infinite transfer.
    SDHC_BLKATTR =
        SDHC_BLKATTR_BLKCNT(MAX_BLKCNT) | SDHC_BLKATTR_BLKSIZE(512);
#endif // defined(__IMXRT1062__)

    if (!cardCommand(CMD18_PGM_XFERTYP,
                     m_highCapacity ? sector : 512 * sector)) {
        return sdError(SD_CARD_ERROR_CMD18);
    }
    return 1;
}

int mk6x_sdio_readStop() { return transferStop(); }

uint32_t mk6x_sdio_sectorCount() { return sdCardCapacity(&m_csd); }

uint32_t mk6x_sdio_status() { return statusCMD13(); }

int mk6x_sdio_stopTransmission(int blocking) {
    m_curState = IDLE_STATE;
    // This fix allows CDIHB to be cleared in Tennsy 3.x without a reset.
    SDHC_PROCTL &= ~SDHC_PROCTL_SABGREQ;
    if (!cardCommand(CMD12_XFERTYP, 0)) {
        return sdError(SD_CARD_ERROR_CMD12);
    }
    if (blocking) {
        if (yieldTimeout(isBusyDat)) {
            return sdError(SD_CARD_ERROR_CMD13);
        }
    }
    return 1;
}

int mk6x_sdio_syncDevice() {
    if (!waitTransferComplete()) {
        return 0;
    }
    if (m_curState != IDLE_STATE) {
        return mk6x_sdio_stopTransmission(1);
    }
    return 1;
}

uint8_t mk6x_sdio_cardtype() {
    return m_version2
               ? m_highCapacity ? SD_CARD_TYPE_SDHC : SD_CARD_TYPE_SD2
               : SD_CARD_TYPE_SD1;
}

int mk6x_sdio_writeData(char *src) {
    const uint32_t* p32;
    /* char *ptr;*/
    uint32_t aligned[512/sizeof(uint32_t)];

    p32 = aligned;
    
    memcpy(aligned, src, 512);

    DBG_IRQSTAT();
    if (!waitTransferComplete()) {
        return 0;
    }
    if (!(SDHC_PRSSTAT & SDHC_PRSSTAT_WTA)) {
        SDHC_PROCTL &= ~SDHC_PROCTL_SABGREQ;
        SDHC_PROCTL |= SDHC_PROCTL_CREQ;
    }
    SDHC_PROCTL |= SDHC_PROCTL_SABGREQ;
    if (waitTimeout(isBusyFifoWrite)) {
        return sdError(SD_CARD_ERROR_WRITE_FIFO);
    }
    for (uint32_t iw = 0; iw < 512 / (4 * FIFO_WML); iw++) {
        while (0 == (SDHC_PRSSTAT & SDHC_PRSSTAT_BWEN)) {
        }
        for (uint32_t i = 0; i < FIFO_WML; i++) {
            SDHC_DATPORT = p32[i];
        }
        p32 += FIFO_WML;
    }
    m_transferActive = 1;
    return 1;
}

int mk6x_sdio_writeSector(uint32_t sector, char *src) {
    char *ptr;
    char aligned[512];
    if (ALIGNMENT_TARGET & (uint32_t) src) {
	ptr = aligned;
	memcpy(aligned, src, 512);
    } else {
	ptr = (src);
    }
    if (SDIOCONFIG_USE_DMA) {
        if (!rdWrSectors(CMD24_DMA_XFERTYP, sector, ptr, 1)) {
            return sdError(SD_CARD_ERROR_CMD24);
        }
    } else {
        if (!waitTransferComplete()) {
            return 0;
        }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        // End transfer with CMD12 if required.
        if ((SDHC_BLKATTR & 0XFFFF0000) == 0) {
            if (!mk6x_sdio_syncDevice()) {
                return 0;
            }
        }
#endif // defined(__MK64FX512__) || defined(__MK66FX1M0__)
        if (m_curState != WRITE_STATE || m_curSector != sector) {
            if (!mk6x_sdio_syncDevice()) {
                return 0;
            }
            if (!mk6x_sdio_writeStart(sector)) {
                return 0;
            }
            m_curSector = sector;
            m_curState  = WRITE_STATE;
        }
        if (!mk6x_sdio_writeData(ptr)) {
            return 0;
        }
        m_curSector++;
    }
    return 1;
}

int mk6x_sdio_writeSectors(uint32_t sector, char *src, size_t n) {
    if (SDIOCONFIG_USE_DMA) {
        char *ptr = (src);
        if (ALIGNMENT_TARGET & (uint32_t) ptr) {
            for (size_t i = 0; i < n; i++, sector++, ptr += 512) {
                if (!mk6x_sdio_writeSector(sector, ptr)) {
                    return 0; // writeSector will set errorCode.
                }
            }
            return 1;
        }
        if (!rdWrSectors(CMD25_DMA_XFERTYP, sector, ptr, n)) {
            return sdError(SD_CARD_ERROR_CMD25);
        }
    } else {
        for (size_t i = 0; i < n; i++) {
            if (!mk6x_sdio_writeSector(sector + i, src + i * 512UL)) {
                return 0;
            }
        }
    }
    return 1;
}

int mk6x_sdio_writeStart(uint32_t sector) {
    if (yieldTimeout(isBusyCMD13)) {
        return sdError(SD_CARD_ERROR_CMD13);
    }
    SDHC_PROCTL &= ~SDHC_PROCTL_SABGREQ;

#if defined(__IMXRT1062__)
    // Infinite transfer.
    SDHC_BLKATTR = SDHC_BLKATTR_BLKSIZE(512);
#else  // defined(__IMXRT1062__)
    // Errata - can't do infinite transfer.
    SDHC_BLKATTR =
        SDHC_BLKATTR_BLKCNT(MAX_BLKCNT) | SDHC_BLKATTR_BLKSIZE(512);
#endif // defined(__IMXRT1062__)
    if (!cardCommand(CMD25_PGM_XFERTYP,
                     m_highCapacity ? sector : 512 * sector)) {
        return sdError(SD_CARD_ERROR_CMD25);
    }
    return 1;
}
//------------------------------------------------------------------------------
int mk6x_sdio_writeStop() { return transferStop(); }

#endif
