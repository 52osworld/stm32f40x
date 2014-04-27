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

static struct rt_messagequeue  rx_mq;
static char msg_pool[1024]; 
struct  rx_msg
{
  rt_device_t dev;
  rt_size_t size;
};


rt_err_t uart_input_proc(rt_device_t dev, rt_size_t size) {
  struct rx_msg msg;
  msg.dev = dev;
  msg.size = size;
  return rt_mq_send(&rx_mq, &msg, sizeof(struct rx_msg));
}

ALIGN(RT_ALIGN_SIZE)
static char thread_uart_recv_stack[512];
struct rt_thread thread_uart_recv;
static void rt_thread_entry_uart_recv(void* parameter)
{
  rt_device_t device;
  rt_err_t result = RT_EOK;
  struct rx_msg msg;  
  static char uart_rx_buffer[64];
  device = rt_device_find("uart1");
  if (device == RT_NULL) {
    rt_kprintf("Device not found!\r\n");
  } else {
    rt_device_set_rx_indicate(device, uart_input_proc);
    result = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
  }
  while(1) {
    result = rt_mq_recv(&rx_mq, &msg, sizeof(struct rx_msg), 50);
    if (result == -RT_ETIMEOUT) {
    
    }
    else if (result == RT_EOK) {
      rt_uint16_t rx_length = 0;
      rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?  
                msg.size : sizeof(uart_rx_buffer) - 1;
      rx_length = rt_device_read(device, 0, &uart_rx_buffer[0], rx_length);
      uart_rx_buffer[rx_length] = '\0'; 
      rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
    }
  }
}

int rt_application_init()
{
  rt_err_t result;   
  result = rt_mq_init(&rx_mq, "mqt", &msg_pool[0], 128 - sizeof(void*), sizeof(msg_pool), RT_IPC_FLAG_FIFO); 
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
  return 0;
}

/*@}*/
