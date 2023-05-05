#include "led.h"
#include "core1.h"
#include <stdint.h>


volatile long g_val = 123;
volatile long g_valu[2];
volatile const long c_val = 321;

volatile uint32_t *SPINLOCK1 = (volatile uint32_t *)(0xd0000000uL + 0x104uL);

static void spin_lock(void)
{
	while (*SPINLOCK1 == 0u) { }
	__sync_synchronize();
}

static void spin_unlock(void)
{
	*SPINLOCK1 = 0;
	__sync_synchronize();
}

volatile long cnt = 0;
void incrfunc(void)
{
	for (volatile long i = 0; i < 1000000; i++) {
		spin_lock();
		cnt++;
		spin_unlock();
	}	
}

void core1_func(void)
{
	incrfunc();
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
	incrfunc();
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
