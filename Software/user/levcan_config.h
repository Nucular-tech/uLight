/*
 * levcan_config.h
 *
 *  Created on: 22 мар. 2018 г.
 *      Author: Vasiliy Sukhoparov (VasiliSk)
 */

#pragma once

//user functions for critical sections
static inline void lc_enable_irq(void) {

	asm volatile ("cpsie i" : : : "memory");
}
static inline void lc_disable_irq(void) {
	asm volatile ("cpsid i" : : : "memory");
}

#ifdef TRACE
//Print debug messages using trace_printf
//#define LEVCAN_TRACE
//You can re-define trace_printf function
//#define trace_printf printf
#endif

#define LC_EXPORT

extern void assert_failed(char *file, uint32_t line);
/* Define configASSERT() to disable interrupts and sit in a loop. */
#define configASSERT(x)     if( ( x ) == 0 ) { assert_failed(__FILE__, __LINE__); for( ;; ); }

#define LEVCAN_FILECLIENT
#define LEVCAN_PARAMETERS_SERVER
#define LEVCAN_PARAMETERS_PARSING
//Float-point support
//#define LEVCAN_USE_FLOAT
#define LEVCAN_EVENTS
//Memory packing, compiler specific
#define LEVCAN_PACKED __attribute__((packed))

//Max device created nodes
#define LEVCAN_MAX_OWN_NODES 1

//Network node table
#define LEVCAN_MAX_TABLE_NODES 5

//Above-driver buffer size. Used to store CAN messages before calling network manager
#define LEVCAN_TX_SIZE 20
#define LEVCAN_RX_SIZE 30

//enable parameters and setup receive buffer size
#define LEVCAN_PARAM_QUEUE_SIZE 5

//Default size for malloc, maximum size for static mem, data size for file i/o
#define LEVCAN_OBJECT_DATASIZE 64
#define LEVCAN_FILE_DATASIZE 512
#define LEVCAN_BUFFER_FILEPRINTF

//File operations timeout for client side
#define LEVCAN_FILE_TIMEOUT 500

//Enable this to use only static memory
#define LEVCAN_MEM_STATIC

#ifdef LEVCAN_MEM_STATIC
//Maximum TX/RX objects. Excl. UDP data <=8byte, this receives in fast mode
#define LEVCAN_OBJECT_SIZE 30
#else
//external malloc functions
#define lcmalloc pvPortMalloc
#define lcfree vPortFree
//extern delay function lcdelay(uint32_t time)
#define lcdelay vTaskDelay
#endif
extern void delay_ms(uint32_t delay);
#define lcdelay delay_ms
