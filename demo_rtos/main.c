/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/gpio.h"


#define mainTASK_PRIORITY		( tskIDLE_PRIORITY + 2 )


static void Task_1(void *pvParameters)
{
	(void)pvParameters;
	vTaskDelay(100 / portTICK_PERIOD_MS);
	for ( ; ; ) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
	}
}

static void Task_2(void *pvParameters)
{
	(void)pvParameters;
	for ( ; ; ) {
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
	}
}

static void prvSetupHardware(void)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
}

int main( void )
{
	prvSetupHardware();

	xTaskCreate(Task_1,		/* The function that implements the task. */
		"Rx", 			/* The text name assigned to the task - for debug only as it is not used by the kernel. */
		1024, 			/* The size of the stack to allocate to the task. */
		NULL, 			/* The parameter passed to the task - not used in this case. */
		mainTASK_PRIORITY, 	/* The priority assigned to the task. */
		NULL);			/* The task handle is not required, so NULL is passed. */

	xTaskCreate(Task_2, "TX", 1024, NULL, mainTASK_PRIORITY, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}

void vApplicationMallocFailedHook( void )
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}

void vApplicationTickHook( void )
{

}
