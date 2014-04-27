/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

//global variables
static struct rt_messagequeue  uart_1_rx_mq;  //message queue
static struct rt_messagequeue  uart_3_rx_mq;  //message queue
static char uart_1_msg_pool[512]; //message pool, size 1024
static char uart_3_msg_pool[512]; //message pool, size 1024

//self-defined message struct
struct  rx_msg
{
  rt_device_t dev;  //received data from dev
  rt_size_t size;  //received data size
};

/***************local funcions***************/
//@fn: uart_1_input_proc
//@brief: call back function of uart rx, then send a msg to message queue
//@para: dev: device
//@para: size: size of data received
//@return: RT_EOK if succeedd
static rt_err_t uart_1_input_proc(rt_device_t dev, rt_size_t size) {
  struct rx_msg msg;
  msg.dev = dev;
  msg.size = size;
  //send a message to queue with device info and size of data received
  return rt_mq_send(&uart_1_rx_mq, &msg, sizeof(struct rx_msg)); 
}

static rt_err_t uart_3_input_proc(rt_device_t dev, rt_size_t size) {
  struct rx_msg msg;
  msg.dev = dev;
  msg.size = size;
  //send a message to queue with device info and size of data received
  return rt_mq_send(&uart_3_rx_mq, &msg, sizeof(struct rx_msg)); 
}

ALIGN(RT_ALIGN_SIZE)
static char thread_uart_recv_stack[512];
struct rt_thread thread_uart_recv;  //receive temp data from zigbee through uart1
static void rt_thread_entry_uart_recv(void* parameter) {
  rt_device_t device;
  rt_err_t result = RT_EOK;
  struct rx_msg msg;
  static char uart_rx_buffer[32];
  device = rt_device_find("uart1");
  if (device == RT_NULL) {
    rt_kprintf("Device not found!\r\n");
  } else {
    rt_device_set_rx_indicate(device, uart_1_input_proc);
    result = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
  }
  while(1) {
    result = rt_mq_recv(&uart_1_rx_mq, &msg, sizeof(struct rx_msg), 50);
    if (result == -RT_ETIMEOUT) {
      //no message, meaning that no UART data received
    }
    else if (result == RT_EOK) {
      rt_uint16_t rx_length = 0;
      rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?  
                  msg.size : sizeof(uart_rx_buffer) - 1;
      rx_length = rt_device_read(device, 0, &uart_rx_buffer[0], rx_length);
      uart_rx_buffer[rx_length] = '\0'; 
      //TODO: process temp data and then display on LCD
      rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
    }
  }
}

static char thread_upper_recv_stack[512];
struct rt_thread thread_upper_recv;  //receive cmd from upper machine through uart3
static void rt_thread_entry_upper_recv(void* parameter) {
  rt_device_t device;
  rt_err_t result = RT_EOK;
  struct rx_msg msg;
  static char uart_rx_buffer[32];
  device = rt_device_find("uart3");
  if (device == RT_NULL) {
    rt_kprintf("Device not found!\r\n");
  } else {
    rt_device_set_rx_indicate(device, uart_3_input_proc);
    result = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
  }
  while(1) {
    result = rt_mq_recv(&uart_3_rx_mq, &msg, sizeof(struct rx_msg), 50);
    if (result == -RT_ETIMEOUT) {
      //no message, meaning that no UART data received
    }
    else if (result == RT_EOK) {
      rt_uint16_t rx_length = 0;
      rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?  
                  msg.size : sizeof(uart_rx_buffer) - 1;
      rx_length = rt_device_read(device, 0, &uart_rx_buffer[0], rx_length);
      //TODO: process cmds from upper machine
      uart_rx_buffer[rx_length] = '\0';
      rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
    }
  }
}
int rt_application_init()
{
  rt_err_t result;   
  result = rt_mq_init(&uart_1_rx_mq,  //message handler
                      "mq_uart1",  //message queue from uart1
                      &uart_1_msg_pool[0], //mem pool
                      128 - sizeof(void*),  //size of each message
                      sizeof(uart_1_msg_pool),  //sizeof uart_1_msg_pool
                      RT_IPC_FLAG_FIFO);  //First In First Out 
  if (result != RT_EOK) {   
    rt_kprintf("init message queue failed.\n");
  } 
  
  result = rt_mq_init(&uart_3_rx_mq,  //message handler
                    "mq_uart1",  //message queue from uart1
                    &uart_3_msg_pool[0], //mem pool
                    128 - sizeof(void*),  //size of each message
                    sizeof(uart_3_msg_pool),  //sizeof uart_1_msg_pool
                    RT_IPC_FLAG_FIFO);  //First In First Out 
  if (result != RT_EOK) {   
    rt_kprintf("init message queue failed.\n");
  }                   
                    
  rt_thread_init(&thread_uart_recv,
                 "uart_recv",
                 rt_thread_entry_uart_recv,
                 RT_NULL,
                 &thread_uart_recv_stack[0],
                 sizeof(thread_uart_recv_stack), 11, 10);
  rt_thread_startup(&thread_uart_recv); 

  rt_thread_init(&thread_upper_recv,
               "upper_recv",
               rt_thread_entry_upper_recv,
               RT_NULL,
               &thread_upper_recv_stack[0],
               sizeof(thread_upper_recv_stack), 11, 10);
  rt_thread_startup(&thread_upper_recv);   
  return 0;
}

/*@}*/
