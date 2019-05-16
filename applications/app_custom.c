#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
static volatile bool stop_now = true;
static volatile bool is_running = false;
// Example thread
static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread

/* 
void app_example_init(void) {

	// Start the example thread
	chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
		NORMALPRIO, example_thread, NULL);
}*/

void app_custom_start(void) {
        if (!is_running)
        {
                is_running = true;
		palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);   //SET TX as input
		palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLDOWN);   //SET RX as input
		chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa), NORMALPRIO, example_thread, NULL);
        }
}

void app_custom_stop(void) {
	
}

 
static THD_FUNCTION(example_thread, arg) 
{
		is_running = true;
		(void)arg;
	 
		chRegSetThreadName("APP_EXAMPLE");
		
	

		

                while (!palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN))
                {
			mc_interface_set_pid_speed(1000.0);
                        chThdSleepMilliseconds(30);
                        timeout_reset();
		}  
               
                mc_interface_set_pid_speed(0.0);
                chThdSleepMilliseconds(1000);
                timeout_reset();
	
		
                while (!palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN))
                {
			mc_interface_set_pid_speed(-1000.0);
                        chThdSleepMilliseconds(30);
                        timeout_reset();
		}  
   
                mc_interface_set_pid_speed(0.0);
                chThdSleepMilliseconds(30);
                timeout_reset();

                is_running = false;
                chThdExit(0);

                
                 
		// Reset the timeout
		

                

}
