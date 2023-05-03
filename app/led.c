#include "led.h"
#include <stdint.h>

// GPIO25 OP Connected to user LED
#define LED_PIN		25

#define	GPIO_FUNC_XIP		0
#define	GPIO_FUNC_SPI		1
#define	GPIO_FUNC_UART		2
#define	GPIO_FUNC_I2C		3
#define	GPIO_FUNC_PWM		4
#define	GPIO_FUNC_SIO		5
#define	GPIO_FUNC_PIO0		6
#define	GPIO_FUNC_PIO1		7
#define	GPIO_FUNC_GPCK		8
#define	GPIO_FUNC_USB		9
#define	GPIO_FUNC_NULL		0x1f


#define REG(addr, base) (*(volatile uint32_t *)(addr + base))

#define SIO_BASE	0xd0000000uL
#define CPUID		REG(0x000, SIO_BASE)
#define GPIO_OUT	REG(0x010, SIO_BASE)
#define GPIO_OUT_SET	REG(0x014, SIO_BASE)
#define GPIO_OUT_CLR	REG(0x018, SIO_BASE)
#define GPIO_OE_SET	REG(0x024, SIO_BASE)
#define GPIO_OE_CLR	REG(0x028, SIO_BASE)

#define IO_BANK0_BASE	0x40014000uL
#define GPIO25_CTRL	REG(0x0cc, IO_BANK0_BASE)

#define CLOCKS_BASE	0x40008000uL
#define CLK_PERI_CTRL	REG(0x48, CLOCKS_BASE)

#define RESETS_BASE	0x4000c000uL
#define RESETS		REG(0x0, CLOCKS_BASE)
#define RESETS_RESET_IO_QSPI_BITS	0x00000040uL
#define RESETS_RESET_PADS_QSPI_BITS	0x00000200uL
#define RESETS_RESET_PLL_USB_BITS	0x00002000uL
#define RESETS_RESET_USBCTRL_BITS	0x01000000uL
#define RESETS_RESET_SYSCFG_BITS	0x00040000uL
#define RESETS_RESET_PLL_SYS_BITS	0x00001000uL
#define RESETS_RESET_BITS		0x01ffffffuL
#define RESETS_RESET_ADC_BITS		0x00000001uL
#define RESETS_RESET_RTC_BITS		0x00008000uL
#define RESETS_RESET_SPI0_BITS		0x00010000uL
#define RESETS_RESET_SPI1_BITS		0x00020000uL
#define RESETS_RESET_UART0_BITS		0x00400000uL
#define RESETS_RESET_UART1_BITS		0x00800000uL
#define RESETS_RESET_USBCTRL_BITS	0x01000000uL


#define REG_ALIAS_CLR_BITS (0x3u << 12u)

static void hw_clear_bits(volatile uint32_t *addr, uint32_t mask) {
    *(volatile uint32_t *)(REG_ALIAS_CLR_BITS | (uintptr_t)addr) = mask;
}

void led_init(void)
{
	// Remove reset from peripherals which are clocked
	hw_clear_bits((volatile uint32_t *)RESETS_BASE, RESETS_RESET_BITS);
	for (volatile long long i = 0; i < 1000; i++) { }

	GPIO_OE_CLR = 1uL << LED_PIN;
	GPIO_OUT_CLR = 1uL << LED_PIN;
	GPIO25_CTRL = GPIO_FUNC_SIO;
	GPIO_OE_SET = 1uL << LED_PIN;
}

void led_on(void)
{
	GPIO_OUT_SET = 1uL << LED_PIN;
}

void led_off(void)
{
	GPIO_OUT_CLR = 1uL << LED_PIN;
}
