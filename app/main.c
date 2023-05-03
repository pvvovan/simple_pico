#include "led.h"
#include "core1.h"


volatile long g_val = 123;
volatile long g_valu[2];
volatile const long c_val = 321;

void core1_func(void)
{
	while (1) {
		for (volatile long long i = 0; i < 100000; i++) { }
		led_off();
		for (volatile long long i = 0; i < 100000; i++) { }
		// led_on();
	}
}

int main()
{
	led_init();

	// // SEV causes an event to be signaled to all cores within a multiprocessor system.
	// // If SEV is implemented, WFE must also be implemented.
	// asm volatile ("sev");
	multicore_launch_core1(&core1_func);

	for ( ; ; ) {
		for (volatile long long i = 0; i < 100000; i++) { }
		// led_off();
		for (volatile long long i = 0; i < 100000; i++) { }
		led_on();

		g_val++;
		g_valu[1] += c_val;
	}

	return 0;
}
