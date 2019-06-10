#include "asf.h"
#include "main.h"
#include <string.h>

#include "OLED1/gfx_mono_ug_2832hsweg04.h"
#include "OLED1/gfx_mono_text.h"
#include "OLED1/sysfont.h"

/************************************************************************/
/* GENERIC DEFINES                                                      */
/************************************************************************/

/************************************************************************/
/* generic globals                                                      */
/************************************************************************/

/************************************************************************/
/*  RTOS    (defines + globals)                                         */
/************************************************************************/

#define TASK_STRING_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_STRING_STACK_PRIORITY        (tskIDLE_PRIORITY)

// Botoes
#define BUT1_PIO           PIOD
#define BUT1_PIO_ID        ID_PIOD
#define BUT1_PIO_IDX       28
#define BUT1_IDX_MASK      (1u << BUT1_PIO_IDX)

#define BUT2_PIO           PIOC
#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO_IDX       31
#define BUT2_IDX_MASK      (1u << BUT2_PIO_IDX)

#define BUT3_PIO           PIOA
#define BUT3_PIO_ID        ID_PIOA
#define BUT3_PIO_IDX       19
#define BUT3_IDX_MASK      (1u << BUT3_PIO_IDX)

// LEDs
#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_IDX_MASK      (1u << LED1_PIO_IDX)

#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_IDX_MASK      (1u << LED2_PIO_IDX)

#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_IDX_MASK      (1u << LED3_PIO_IDX)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}
extern void vApplicationIdleHook(void)
{
	
}
extern void vApplicationTickHook(void)
{
}
extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

SemaphoreHandle_t xSemaphoreLED;

QueueHandle_t xQueueUART;

QueueHandle_t xQueueOLED;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

int protocol_check_led(char *string);
int io_init(void);
void led_on(uint id, uint on);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* IRQS / callbacks                                                     */
/************************************************************************/

void USART1_Handler(void){
	char b;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // freeRTOs
  uint32_t ret = usart_get_status(CONF_UART);  // ACK IRQ
  // Por qual motivo entrou na interrupçcao ? Pode ser varios!
  //  1. RX: Dado disponível para leitura
  if(ret & US_IER_RXRDY){
    // LER DADO DA UART
	// printf("funfou1111!!!!!!!!!!!!!11");
	usart_serial_getchar(CONF_UART, &b);
	
    // ENVIAR VIA FILA
	xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueUART, &b, &xHigherPriorityTaskWoken);
  } 
}

void button0_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}

void button1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}

void button2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_string(void *pvParameters){
    xQueueUART = xQueueCreate( 52, sizeof( char ) );
	xQueueOLED = xQueueCreate( 2, sizeof( char[52] ) );

	//char envia[52];
  char b[52];
  uint i = 0;
  while(1){
    // if(usart_serial_is_rx_ready(CONF_UART)){
      // usart_serial_getchar(CONF_UART, &b[i]);
	if (xQueueReceive (xQueueUART, &b[i], ( TickType_t ) 0 )){
			if(b[i]=='\n'){
				b[i] = NULL;
				i = 0;
				printf("recebido: %s\n", b);
				printf("%d", protocol_check_led(b));
				xQueueSend(xQueueOLED, b, 10);
				if (protocol_check_led(b)){
					BaseType_t xHigherPriorityTaskWoken = pdTRUE;
					xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
				}
				
			}else{
				i++;
			}               
    }    
  } 
}  


void task_io(void *pvParameters){
	xSemaphoreLED = xSemaphoreCreateBinary();
	if (xSemaphoreLED == NULL){
		printf("falha em criar o semaforo \n");
	}
	io_init();
	int entrou = 0;
	while(1){
		if(entrou){
			led_on(0, 1);
			led_on(1, 1);
			led_on(2, 1);
			vTaskDelay(100);
			led_on(0, 0);
			led_on(1, 0);
			led_on(2, 0);
			vTaskDelay(100);
		} else {
			led_on(0, 1);
			led_on(1, 1);
			led_on(2, 1);
			vTaskDelay(100);
		}
		if( xSemaphoreTakeFromISR(xSemaphoreLED, 0)){
			entrou = !entrou;
		}
	}
}

void task_oled1(void *pvParameters){
	gfx_mono_ssd1306_init();
	// gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	char string[52];
	while (1){
		if (xQueueReceive (xQueueOLED, string, ( TickType_t ) 0)){
			printf("%s", string);
			gfx_mono_draw_string(string, 50, 16, &sysfont);
		}
	}
	
}  




/************************************************************************/
/* CONFIGS/ INITS                                                       */
/************************************************************************/

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};


	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	usart_enable_interrupt(CONF_UART, US_IER_RXRDY);
	NVIC_EnableIRQ(CONSOLE_UART_ID);
	NVIC_SetPriority(CONSOLE_UART_ID, 5);
}


/************************************************************************/
/* functions                                                            */
/************************************************************************/



int protocol_check_led(char *string){
	return !strcmp(string, "LEDS");
}


int io_init(void){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);
	

	
	 // Inicializa clock do periférico PIO responsavel pelo botao
	 pmc_enable_periph_clk(BUT1_PIO_ID);

	 // Configura PIO para lidar com o pino do botão como entrada
	 // com pull-up
	 pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	 // Configura interrupção no pino referente ao botao e associa
	 // função de callback caso uma interrupção for gerada
	 // a função de callback é a: but_callback()
	 pio_handler_set(BUT1_PIO,
	 BUT1_PIO_ID,
	 BUT1_IDX_MASK,
	 PIO_IT_FALL_EDGE,
	 button0_callback);

	 // Ativa interrupção
	 pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);

	 // Configura NVIC para receber interrupcoes do PIO do botao
	 // com prioridade 4 (quanto mais próximo de 0 maior)
	 NVIC_EnableIRQ(BUT1_PIO_ID);
	 NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	 
	 // Inicializa clock do periférico PIO responsavel pelo botao
	 pmc_enable_periph_clk(BUT2_PIO_ID);

	 // Configura PIO para lidar com o pino do botão como entrada
	 // com pull-up
	 pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	 // Configura interrupção no pino referente ao botao e associa
	 // função de callback caso uma interrupção for gerada
	 // a função de callback é a: but_callback()
	 pio_handler_set(BUT2_PIO,
	 BUT2_PIO_ID,
	 BUT2_IDX_MASK,
	 PIO_IT_FALL_EDGE,
	 button1_callback);

	 // Ativa interrupção
	 pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);

	 // Configura NVIC para receber interrupcoes do PIO do botao
	 // com prioridade 4 (quanto mais próximo de 0 maior)
	 NVIC_EnableIRQ(BUT2_PIO_ID);
	 NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4

	 
	 // Inicializa clock do periférico PIO responsavel pelo botao
	 pmc_enable_periph_clk(BUT3_PIO_ID);

	 // Configura PIO para lidar com o pino do botão como entrada
	 // com pull-up
	 pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	 // Configura interrupção no pino referente ao botao e associa
	 // função de callback caso uma interrupção for gerada
	 // a função de callback é a: but_callback()
	 pio_handler_set(BUT3_PIO,
	 BUT3_PIO_ID,
	 BUT3_IDX_MASK,
	 PIO_IT_FALL_EDGE,
	 button2_callback);

	 // Ativa interrupção
	 pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);

	 // Configura NVIC para receber interrupcoes do PIO do botao
	 // com prioridade 4 (quanto mais próximo de 0 maior)
	 NVIC_EnableIRQ(BUT3_PIO_ID);
	 NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
	 
	
}
  
void led_on(uint id, uint on){
	if (on){
		if (id == 0){
			pio_set(LED1_PIO, LED1_IDX_MASK);
		} else if (id == 1){
			pio_set(LED2_PIO, LED2_IDX_MASK);
		} else {
			pio_set(LED3_PIO, LED3_IDX_MASK);
		}
	} else {
		if (id == 0){
			pio_clear(LED1_PIO, LED1_IDX_MASK);
		} else if (id == 1){
			pio_clear(LED2_PIO, LED2_IDX_MASK);
		} else {
			pio_clear(LED3_PIO, LED3_IDX_MASK);
		}
	}
	
}


/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
   
	if (xTaskCreate(task_string, "string", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	
	if (xTaskCreate(task_io, "io", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}
	
	if (xTaskCreate(task_oled1, "oled", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Wifi task\r\n");
	}

	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
