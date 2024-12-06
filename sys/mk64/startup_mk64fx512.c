/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifdef KERNEL

#include <machine/intr.h>
#include <machine/machparam.h>
#include <machine/kinetis.h>
#include <machine/mk64fx512.h>
#include <machine/teensy.h>

// Flash Security Setting. On Teensy 3.2, you can lock the MK20 chip to prevent
// ( The same applies to the Teensy 3.5 and Teensy 3.6 for their processors )
// anyone from reading your code.  You CAN still reprogram your Teensy while
// security is set, but the bootloader will be unable to respond to auto-reboot
// requests from Arduino. Pressing the program button will cause a full chip
// erase to gain access, because the bootloader chip is locked out.  Normally,
// erase occurs when uploading begins, so if you press the Program button
// accidentally, simply power cycling will run your program again.  When
// security is locked, any Program button press causes immediate full erase.
// Special care must be used with the Program button, because it must be made
// accessible to initiate reprogramming, but it must not be accidentally
// pressed when Teensy Loader is not being used to reprogram.  To set lock the
// security change this to 0xDC.  Teensy 3.0 and 3.1 do not support security lock.
#define FSEC 0xDE

// Flash Options
#define FOPT 0xF9

extern uint32_t _stext;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;
extern uint32_t __user_data_start;
extern uint32_t __user_data_end;
extern uint32_t u;
//extern void __init_array_start(void);
//extern void __init_array_end(void);


#if defined(KINETISK)
#define F_TIMER F_BUS
#elif defined(KINETISL)

#if F_CPU > 16000000
#define F_TIMER (F_PLL/2)
#else 
#define F_TIMER (F_PLL)
#endif//Low Power

#endif

#if F_TIMER == 128000000
#define DEFAULT_FTM_MOD (65536 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 120000000
#define DEFAULT_FTM_MOD (61440 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 108000000
#define DEFAULT_FTM_MOD (55296 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 96000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 90000000
#define DEFAULT_FTM_MOD (46080 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 80000000
#define DEFAULT_FTM_MOD (40960 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 72000000
#define DEFAULT_FTM_MOD (36864 - 1)
#define DEFAULT_FTM_PRESCALE 2
#elif F_TIMER == 64000000
#define DEFAULT_FTM_MOD (65536 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 60000000
#define DEFAULT_FTM_MOD (61440 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 56000000
#define DEFAULT_FTM_MOD (57344 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 54000000
#define DEFAULT_FTM_MOD (55296 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 48000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 40000000
#define DEFAULT_FTM_MOD (40960 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 36000000
#define DEFAULT_FTM_MOD (36864 - 1)
#define DEFAULT_FTM_PRESCALE 1
#elif F_TIMER == 24000000
#define DEFAULT_FTM_MOD (49152 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 16000000
#define DEFAULT_FTM_MOD (32768 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 8000000
#define DEFAULT_FTM_MOD (16384 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 4000000
#define DEFAULT_FTM_MOD (8192 - 1)
#define DEFAULT_FTM_PRESCALE 0
#elif F_TIMER == 2000000
#define DEFAULT_FTM_MOD (4096 - 1)
#define DEFAULT_FTM_PRESCALE 0
#endif

extern int main (void);
void ResetHandler(void);

void fault_isr(void)
{
	while (1) {
		// keep polling some communication while in fault
		// mode, so we don't completely die.
		if (SIM_SCGC4 & SIM_SCGC4_USBOTG) usb_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART0) uart0_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART1) uart1_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART2) uart2_status_isr();
	}
}

void unused_isr(void)
{
    teensy_gpio_led_value(0xAA);
    fault_isr();
}

void nmi_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void hard_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void memmanage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void bus_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void usage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void svcall_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void debugmonitor_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pendablesrvreq_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void systick_isr(void);

void dma_ch0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch4_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch5_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch6_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch7_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch8_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch9_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch10_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch11_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch12_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch13_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch14_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch15_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void mcm_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void randnum_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void flash_cmd_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void flash_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void low_voltage_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void wakeup_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void watchdog_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void sdhc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_timer_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void enet_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void i2s0_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void uart0_lon_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void lpuart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void adc0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void adc1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmt_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void rtc_alarm_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void rtc_seconds_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pit_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pdb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_charge_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void usbhs_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usbhs_phy_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void dac0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dac1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tsi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void mcg_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void lptmr_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porta_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porte_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portcd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void software_isr(void)		__attribute__ ((weak, alias("unused_isr")));

__attribute__ ((section(".dmabuffers"), used, aligned(512)))
void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);

__attribute__ ((section(".vectors"), used))
void (* const _VectorsFlash[NVIC_NUM_INTERRUPTS+16])(void) =
{
  (void (*)(void))((unsigned long) &_estack),	//  0 ARM: Initial Stack Pointer
	ResetHandler,					//  1 ARM: Initial Program Counter
	nmi_isr,					//  2 ARM: Non-maskable Interrupt (NMI)
	hard_fault_isr,					//  3 ARM: Hard Fault
	memmanage_fault_isr,				//  4 ARM: MemManage Fault
	bus_fault_isr,					//  5 ARM: Bus Fault
	usage_fault_isr,				//  6 ARM: Usage Fault
	fault_isr,					//  7 --
	fault_isr,					//  8 --
	fault_isr,					//  9 --
	fault_isr,					// 10 --
	svcall_isr,					// 11 ARM: Supervisor call (SVCall)
	debugmonitor_isr,				// 12 ARM: Debug Monitor
	fault_isr,					// 13 --
	pendablesrvreq_isr,				// 14 ARM: Pendable req serv(PendableSrvReq)
	systick_isr,					// 15 ARM: System tick timer (SysTick)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_ch4_isr,					// 20 DMA channel 4 transfer complete
	dma_ch5_isr,					// 21 DMA channel 5 transfer complete
	dma_ch6_isr,					// 22 DMA channel 6 transfer complete
	dma_ch7_isr,					// 23 DMA channel 7 transfer complete
	dma_ch8_isr,					// 24 DMA channel 8 transfer complete
	dma_ch9_isr,					// 25 DMA channel 9 transfer complete
	dma_ch10_isr,					// 26 DMA channel 10 transfer complete
	dma_ch11_isr,					// 27 DMA channel 11 transfer complete
	dma_ch12_isr,					// 28 DMA channel 12 transfer complete
	dma_ch13_isr,					// 29 DMA channel 13 transfer complete
	dma_ch14_isr,					// 30 DMA channel 14 transfer complete
	dma_ch15_isr,					// 31 DMA channel 15 transfer complete
	dma_error_isr,					// 32 DMA error interrupt channel
	mcm_isr,					// 33 MCM
	flash_cmd_isr,					// 34 Flash Memory Command complete
	flash_error_isr,				// 35 Flash Read collision
	low_voltage_isr,				// 36 Low-voltage detect/warning
	wakeup_isr,					// 37 Low Leakage Wakeup
	watchdog_isr,					// 38 Both EWM and WDOG interrupt
	randnum_isr,					// 39 Random Number Generator
	i2c0_isr,					// 40 I2C0
	i2c1_isr,					// 41 I2C1
	spi0_isr,					// 42 SPI0
	spi1_isr,					// 43 SPI1
	i2s0_tx_isr,					// 44 I2S0 Transmit
	i2s0_rx_isr,					// 45 I2S0 Receive
	unused_isr,					// 46 --
	uart0_status_isr,				// 47 UART0 status
	uart0_error_isr,				// 48 UART0 error
	uart1_status_isr,				// 49 UART1 status
	uart1_error_isr,				// 50 UART1 error
	uart2_status_isr,				// 51 UART2 status
	uart2_error_isr,				// 52 UART2 error
	uart3_status_isr,				// 53 UART3 status
	uart3_error_isr,				// 54 UART3 error
	adc0_isr,					// 55 ADC0
	cmp0_isr,					// 56 CMP0
	cmp1_isr,					// 57 CMP1
	ftm0_isr,					// 58 FTM0
	ftm1_isr,					// 59 FTM1
	ftm2_isr,					// 60 FTM2
	cmt_isr,					// 61 CMT
	rtc_alarm_isr,					// 62 RTC Alarm interrupt
	rtc_seconds_isr,				// 63 RTC Seconds interrupt
	pit0_isr,					// 64 PIT Channel 0
	pit1_isr,					// 65 PIT Channel 1
	pit2_isr,					// 66 PIT Channel 2
	pit3_isr,					// 67 PIT Channel 3
	pdb_isr,					// 68 PDB Programmable Delay Block
 	usb_isr,					// 69 USB OTG
	usb_charge_isr,					// 70 USB Charger Detect
	unused_isr,					// 71 --
	dac0_isr,					// 72 DAC0
	mcg_isr,					// 73 MCG
	lptmr_isr,					// 74 Low Power Timer
	porta_isr,					// 75 Pin detect (Port A)
	portb_isr,					// 76 Pin detect (Port B)
	portc_isr,					// 77 Pin detect (Port C)
	portd_isr,					// 78 Pin detect (Port D)
	porte_isr,					// 79 Pin detect (Port E)
	software_isr,					// 80 Software interrupt
	spi2_isr,					// 81 SPI2
	uart4_status_isr,				// 82 UART4 status
	uart4_error_isr,				// 83 UART4 error
	uart5_status_isr,				// 84 UART4 status
	uart5_error_isr,				// 85 UART4 error
	cmp2_isr,					// 86 CMP2
	ftm3_isr,					// 87 FTM3
	dac1_isr,					// 88 DAC1
	adc1_isr,					// 89 ADC1
	i2c2_isr,					// 90 I2C2
	can0_message_isr,				// 91 CAN OR'ed Message buffer (0-15)
	can0_bus_off_isr,				// 92 CAN Bus Off
	can0_error_isr,					// 93 CAN Error
	can0_tx_warn_isr,				// 94 CAN Transmit Warning
	can0_rx_warn_isr,				// 95 CAN Receive Warning
	can0_wakeup_isr,				// 96 CAN Wake Up
	sdhc_isr,					// 97 SDHC
	enet_timer_isr,					// 98 Ethernet IEEE1588 Timers
	enet_tx_isr,					// 99 Ethernet Transmit
	enet_rx_isr,					// 100 Ethernet Receive
	enet_error_isr,					// 101 Ethernet Error
};


__attribute__ ((section(".flashconfig"), used))
const uint8_t flashconfigbytes[16] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, FSEC, FOPT, 0xFF, 0xFF
};


// Automatically initialize the RTC.  When the build defines the compile
// time, and the user has added a crystal, the RTC will automatically
// begin at the time of the first upload.
#ifndef TIME_T
#define TIME_T 1349049600 // default 1 Oct 2012 (never used, Arduino sets this)
#endif
extern void *__rtc_localtime; // Arduino build process sets this
extern void rtc_set(unsigned long t);

/**
 * possibly terrible idea: use one of the timers for generating interrupts that
 * keep the USB console UART going. as of now, these are not used for anything.
 *
 * clock for each timer is selectable:
 *
 *  - system clock
 *  - fixed-freq clock (the 32,768 kHz one?)
 *  - external clock
 *
 * the prescaler can divide the clock by 1, 2, 4, ..., 128. then there's a free-
 * running counter of 16 bits width with settable initial and final values, and
 * it can run either up or up-down. channels are for using this to control other
 * peripherals and look really interesting for stuff other than this where I just
 * want to trigger an interrupt constantly.
 *
 * "deadtime insertion"? "match triggers"?
 *
 * interrupts! these can be generated:
 *
 *  - per channel
 *  - when a counter overflows (this we want)
 *  - on fault detection
 *
 * okay so, the registers!
 *
 * FTMx_SC - status & control
 *
 *  7  TOF    timer overflow - this becomes 1 once it overflows
 *  6  TOIE   timer overflow interrupt enable
 *  5  CPWMS  center-aligned PWM mode select
 * 4,3 CLKS   clock source selection
 *             00: none
 *             01: system clock
 *             10: fixed-freq clock
 *             11: external clock
 * 2-0 PS     prescaler factor selection
 *             000: divide by 1 (also known as don't divide)
 *             001: divide by 2
 *             010: divide by 4
 *              ...
 *             111: divide by 128
 *
 * FTMx_CNT - the counter itself
 *
 *  0-15      the count
 *
 * FTMx_MOD - the counter modulo value
 *
 *  0-15      the modulo value. write CNT first, then this.
 *
 * FTMx_CnSC - channel status & control
 *
 *  7  CHF    channel flag
 *  6  CHIE   channel interrupt enable
 *  5  MSB    mode select 
 *  4  MSA     -"-
 *  3  ELSB    -"-
 *  2  ELSA    -"-
 *  1         reserved
 *  0  DMA    enable DMA
 *
 * FTMx_CnV - channel value
 *
 *  0-15     the captured or otherwise conjured channel value.
 *
 * FTMx_CNTIN - counter initial value
 *
 *  0-15     ...yes
 *
 * FTMx_STATUS - capture & compare status
 *
 *  0-7      copies of each channel's flag aka CHF
 *
 * FTMx_MODE - features mode selection
 *
 *  7  FAULTIE fault interrupt enable
 *  6  FAULTM  fault control mode
 *  5   -"-     00: disabled for all channels
 *              01: enabled for even-numbered channels, manual clear
 *              10: enabled for odd-numbered channels, manual clear
 *              11: enabled for all channels, auto clear
 *  4  CAPTEST capture test mode enable
 *  3  PWMSYNC ...this must sound phat
 *  2  WPDIS   write protection disable
 *  1  INIT    initialize the channels output
 *  0  FTMEN   enable FTM so that all features are available
 *
 * ...there is too much of this. I just want a timer, let's say 10 times a second?
 * with 32,768 kHz... hm. this'd make for a 2 Hz thing. with 120 MHz... that is a lot.
 * it overflows at ~1831 Hz as is, so apply full prescaler to divide that by 128 and
 * end up with 14,3 Hz. or possibly half of that if "system clock" is the bus clock.
 * let's see. okay, this could've been done with setting the initial count etc, but
 * for the first try, the easiest settings are the best.
 *
 *  FTMx_SC: TOIE=1, CLKS=01, PS=111
 *
 *  other: QUADEN=0, CPWMS=0  <- these mean "count up"
 *  free-run mode:    FTMEN == 0 && (MOD == 0x0000 || MOD == 0xFFFF)
 *          or:       FTMEN == 1 && (QUADEN = 0
 *                                   && CPWMS == 0
 *                                   && CNTIN == 0x0000
 *                                   && MOD = 0xFFFF)
 *
 */
void flextimer_init(void) {
    #if 0
	FTM0_CNT = 0;
	FTM0_MOD = DEFAULT_FTM_MOD;
	FTM0_C0SC = 0x28; // MSnB:MSnA = 10, ELSnB:ELSnA = 10
	FTM0_C1SC = 0x28;
	FTM0_C2SC = 0x28;
	FTM0_C3SC = 0x28;
	FTM0_C4SC = 0x28;
	FTM0_C5SC = 0x28;
	#endif

	
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	#if 0
	FTM0_C6SC = 0x28;
	FTM0_C7SC = 0x28;
	#endif
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	FTM3_C0SC = 0x28;
	FTM3_C1SC = 0x28;
	FTM3_C2SC = 0x28;
	FTM3_C3SC = 0x28;
	FTM3_C4SC = 0x28;
	FTM3_C5SC = 0x28;
	FTM3_C6SC = 0x28;
	FTM3_C7SC = 0x28;
#endif
	/*	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE); */
	FTM1_CNT = 0;
	FTM1_MOD = DEFAULT_FTM_MOD;
	FTM1_C0SC = 0x28;
	FTM1_C1SC = 0x28;
	FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)
	FTM2_CNT = 0;
	FTM2_MOD = DEFAULT_FTM_MOD;
	FTM2_C0SC = 0x28;
	FTM2_C1SC = 0x28;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	FTM3_CNT = 0;
	FTM3_MOD = DEFAULT_FTM_MOD;
	FTM3_C0SC = 0x28;
	FTM3_C1SC = 0x28;
	FTM3_SC = FTM_SC_CLKS(1) | FTM_SC_PS(DEFAULT_FTM_PRESCALE);
#endif

#define UART_POLLING_INTERRUPT
#ifdef UART_POLLING_INTERRUPT
	FTM0_CNTIN = 1;
	/* ftminterval */
	FTM0_MOD = 0x2FFF; /* adjust this to adjust the firing interval.
	                   * 0xFFF    hm... 1 Hz < this < 10 Hz
			   */
	FTM0_SC = FTM_SC_TOIE | FTM_SC_CLKS(0b10) | FTM_SC_CPWMS | FTM_SC_PS(0);
#endif	
}

void early_irq_init(void) {
  /**
   * get systick going at priority 32. machdep.c:startup() will adjust the priority.
   */
  SYST_RVR = (F_CPU / 1000) - 1;
  SYST_CVR = 0;
  SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
  SCB_SHPR3 = 0x20000000;

  arm_enable_interrupts();
}


#if defined(__PURE_CODE__) || !defined(__OPTIMIZE__) || defined(__clang__) || 0
// cases known to compile too large for 0-0x400 memory region
__attribute__ ((optimize("-Os")))
#else
// hopefully all others fit into startup section (below 0x400)
__attribute__ ((section(".startup"),optimize("-Os")))
#endif
void ResetHandler(void)
{
	uint32_t *src = &_etext;
	uint32_t *dest = &_sdata;
	unsigned int i;

	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	asm volatile ("nop");
	asm volatile ("nop");

	asm volatile ("ldr     sp, =_estack");

	WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;

	memset(&u, 0, 0x800);

	/* turn on double-word stack aligment at first possible moment */
	#if 0
	SCB_CCR |= SCB_CCR_STKALIGN_MASK; 
	#else
	/* ... lets get back to that someday */
	SCB_CCR &= ~SCB_CCR_STKALIGN_MASK;
	SCB_CCR &= ~(1<<3);
	SCB_CCR &= ~(1<<8);
	#endif

	SCB_CPACR = 0x00F00000;

	SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
	SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;

#if defined(KINETISK) && !defined(__MK66FX1M0__)
	// If the RTC oscillator isn't enabled, get it started early.
	// But don't do this early on Teensy 3.6 - RTC_CR depends on 3.3V+VBAT
	// which may be ~0.4V "behind" 3.3V if the power ramps up slowly.
	if (!(RTC_CR & RTC_CR_OSCE)) {
		RTC_SR = 0;
		RTC_CR = RTC_CR_SC16P | RTC_CR_SC4P | RTC_CR_OSCE;
	}
#endif
#if 0
	// release I/O pins hold, if we woke up from VLLS mode
	if (PMC_REGSC & PMC_REGSC_ACKISO) PMC_REGSC |= PMC_REGSC_ACKISO;
#endif

	// since this is a write once register, make it visible to all F_CPU's
	// so we can into other sleep modes in the future at any speed
	/* these settings allow using all power modes the chip supports. */
	SMC_PMPROT = SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;
	
	/* src and dest have been initialized at declaration as follows:
	 *
	 * src = &_etext, dest == &_sdata
	 *
	 * so, what teensyduino does:
	 *
	 * 1. copy from _etext onwargs to range _sdata-_edata. 
	 * 2. zero-fill the bss segment
	 *
	 * agh how can I be this daft. after the text is what's supposed to end up in the
	 * data segment and this is what is copied.
	 *
	 * what the stm32 port does:
	 *
	 * 1. zero-fill from _sdata to u_end.
	 * 2. copy the range from _sidata to _sdata? things are laid out so that
	 *    _sidata == _etext ... so are they actually doing ... nngh. apparently
	 *    they "copy the data segment initializers from flash to ram" but ...
	 *    there is a reason for this.
	 * 3. zero-fill the bss segment
	 *
	 * what breaks this? the distance from sdata to edata is less than 0x200 but
	 * ... this can't be right.
	 *
	 * well, guess what? it wasn't. 
	 *
	 */

	while (dest < &_edata) *dest++ = *src++;

	/*
	 * .... okay, boot once the above disabled, then enable and reboot... 
	 */

	dest = &_sbss;
	while (dest < &_ebss) *dest++ = 0;
	/*
	 * while at it, let's zero-fill the high RAM too.
	 * on second thought, let's pattern-fill instead.
	 */
	dest = &__user_data_start;
	while (dest < &__user_data_end) *dest++ = 0x6502cafe;
	
	/* copy interrupt vector table from flash to RAM, set the "default" priority
	 * to the lowest level and switch over to the RAM copy of the vector table.
	 */
	for (i=0; i < NVIC_NUM_INTERRUPTS + 16; i++) _VectorsRam[i] = _VectorsFlash[i];
#if defined(SET_DEFAULT_INTERRUPT_PRIORITY) || defined (SIMPLE_INTERRUPTS)
	for (i=0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, SPL_DEFAULT);
#endif
	SCB_VTOR = (uint32_t)_VectorsRam;


	
	// hardware always starts in FEI mode
	//  C1[CLKS] bits are written to 00
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] bit is written to 0
	// MCG_SC[FCDIV] defaults to divide by two for internal ref clock
	// I tried changing MSG_SC to divide by 1, it didn't work for me

	// enable capacitors for crystal
	OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
	// enable osc, 8-32 MHz range, low power mode
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
	// switch to crystal as clock source, FLL input = 16 MHz / 512
	MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
	// wait for crystal oscillator to begin
	while ((MCG_S & MCG_S_OSCINIT0) == 0) ;
	// wait for FLL to use oscillator
	while ((MCG_S & MCG_S_IREFST) != 0) ;
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) ;

	// now in FBE mode
	//  C1[CLKS] bits are written to 10
	//  C1[IREFS] bit is written to 0
	//  C1[FRDIV] must be written to divide xtal to 31.25-39 kHz
	//  C6[PLLS] bit is written to 0
	//  C2[LP] is written to 0

	// if we need faster than the crystal, turn on the PLL
    #if F_CPU == 72000000
	MCG_C5 = MCG_C5_PRDIV0(5);		 // config PLL input for 16 MHz Crystal / 6 = 2.667 Hz
    #else
	MCG_C5 = MCG_C5_PRDIV0(3);		 // config PLL input for 16 MHz Crystal / 4 = 4 MHz
    #endif
    #if F_CPU == 168000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(18); // config PLL for 168 MHz output
    #elif F_CPU == 144000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(12); // config PLL for 144 MHz output
    #elif F_CPU == 120000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(6); // config PLL for 120 MHz output
    #elif F_CPU == 72000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(3); // config PLL for 72 MHz output
    #elif F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0); // config PLL for 96 MHz output
    #elif F_CPU > 16000000
    #error "This clock speed isn't supported..."
    #endif


	// wait for PLL to start using xtal as its input
	while (!(MCG_S & MCG_S_PLLST)) ;
	// wait for PLL to lock
	while (!(MCG_S & MCG_S_LOCK0)) ;
	// now we're in PBE mode

	// now program the clock dividers
#if F_CPU == 256000000
	// config divisors: 256 MHz core, 64 MHz bus, 32 MHz flash, USB = IRC48M
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 64000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 128000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(7);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);
#elif F_CPU == 240000000
	// config divisors: 240 MHz core, 60 MHz bus, 30 MHz flash, USB = 240 / 5
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 80000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 120000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(7);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(4);
#elif F_CPU == 216000000
	// config divisors: 216 MHz core, 54 MHz bus, 27 MHz flash, USB = IRC48M
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 54000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 108000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(7);	
	
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);
#elif F_CPU == 192000000
	// config divisors: 192 MHz core, 48 MHz bus, 27.4 MHz flash, USB = 192 / 4
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 64000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 96000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(6);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(3);
#elif F_CPU == 180000000
	// config divisors: 180 MHz core, 60 MHz bus, 25.7 MHz flash, USB = IRC48M
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 90000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(6);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);
#elif F_CPU == 168000000
	// config divisors: 168 MHz core, 56 MHz bus, 28 MHz flash, USB = 168 * 2 / 7
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(5);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(6) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 144000000
	// config divisors: 144 MHz core, 48 MHz bus, 28.8 MHz flash, USB = 144 / 3
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(4);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(4);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2);
#elif F_CPU == 120000000
	// config divisors: 120 MHz core, 60 MHz bus, 24 MHz flash, USB = 128 * 2 / 5
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(4);
	#elif F_BUS == 120000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(4);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(4) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 96000000
	// config divisors: 96 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(3);
	#elif F_BUS == 96000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(3);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
#elif F_CPU == 72000000
	// config divisors: 72 MHz core, 36 MHz bus, 24 MHz flash, USB = 72 * 2 / 3
	#if F_BUS == 36000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(2);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(2);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 48000000
	// config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) |  SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1);
  #endif
#elif F_CPU == 24000000
	// config divisors: 24 MHz core, 24 MHz bus, 24 MHz flash, USB = 96 / 2
	#if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
	#elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
	#endif
#elif F_CPU == 16000000
	// config divisors: 16 MHz core, 16 MHz bus, 16 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV3(0) | SIM_CLKDIV1_OUTDIV4(0);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 8000000
	// config divisors: 8 MHz core, 8 MHz bus, 8 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(1);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 4000000
	// config divisors: 4 MHz core, 4 MHz bus, 2 MHz flash
	// since we are running from external clock 16MHz
	// fix outdiv too -> cpu 16/4, bus 16/4, flash 16/4
	// here we can go into vlpr?
	// config divisors: 4 MHz core, 4 MHz bus, 4 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 2000000
	// since we are running from the fast internal reference clock 4MHz
	// but is divided down by 2 so we actually have a 2MHz, MCG_SC[FCDIV] default is 2
	// fix outdiv -> cpu 2/1, bus 2/1, flash 2/2
	// config divisors: 2 MHz core, 2 MHz bus, 1 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(1);
  #elif defined(KINETISL)
	// config divisors: 2 MHz core, 1 MHz bus, 1 MHz flash
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1);
  #endif
#else
#error "Error, F_CPU must be 256, 240, 216, 192, 180, 168, 144, 120, 96, 72, 48, 24, 16, 8, 4, or 2 MHz"
#endif

	
	// switch to PLL as clock source, FLL input = 16 MHz / 512
	MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
	// wait for PLL clock to be used
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3)) ;
	// now we're in PEE mode
	// trace is CPU clock, CLKOUT=OSCERCLK0
	// USB uses PLL clock
	SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);

	early_irq_init();

	flextimer_init();

	
#if defined(KINETISK)
	// RTC initialization
	if (RTC_SR & RTC_SR_TIF) {
		// this code will normally run on a power-up reset
		// when VBAT has detected a power-up.  Normally our
		// compiled-in time will be stale.  Write a special
		// flag into the VBAT register file indicating the
		// RTC is set with known-stale time and should be
		// updated when fresh time is known.
		#if ARDUINO >= 10600
		rtc_set((uint32_t)&__rtc_localtime);
		#else
		rtc_set(TIME_T);
		#endif
		*(uint32_t *)0x4003E01C = 0x5A94C3A5;
	}
	if ((RCM_SRS0 & RCM_SRS0_PIN) && (*(uint32_t *)0x4003E01C == 0x5A94C3A5)) {
		// this code should run immediately after an upload
		// where the Teensy Loader causes the Mini54 to reset.
		// Our compiled-in time will be very fresh, so set
		// the RTC with this, and clear the VBAT resister file
		// data so we don't mess with the time after it's been
		// set well.
		#if ARDUINO >= 10600
		rtc_set((uint32_t)&__rtc_localtime);
		#else
		rtc_set(TIME_T);
		#endif
		*(uint32_t *)0x4003E01C = 0;
	}
#endif

	/**
	 * wait just a bit before jumping in.
	 *
	 * (without this you may not get USB)
	 */

	main();
	asm volatile("cpsid i");
	asm volatile("movs r0, #0");
	asm volatile("msr BASEPRI, r0");
	asm volatile("ldr r0, =__user_data_end");
	asm volatile("msr PSP, r0");
	asm volatile("isb");
	asm volatile("dsb");
	asm volatile("mrs r0, CONTROL"); /* ...thus ARM is guaranteed to start in supervisor mode?
				 * I suppose that's the only sensible way.
				 */
	asm volatile("orrs r0, r0, #0x1");
	asm volatile("orrs r0, r0, #0x2");
	asm volatile("cpsie i");
	asm volatile("msr CONTROL, r0");
	asm volatile("isb");
	asm volatile("dsb");

	asm volatile("ldr lr,=__user_data_start+1");
	asm volatile("bx lr");

	/* fault blinks @~2 Hz, spl counts 0-7 */
	
        while (1) {
	    volatile unsigned int i;
	    volatile unsigned int j;
	    for (i=0; i < 0x1ffff; i++)
		;

	    teensy_gpio_led_spl(j % 8);
	    
	    led_fault(j % 2);

	    i=0;
	    j++;
        };
}

#endif /* KERNEL */
