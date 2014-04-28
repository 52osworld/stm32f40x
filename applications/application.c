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

//global variables [AA][70][00 0A][CC 33 C3 3C]
static char lcd_background_pic[]={0xAA, 0x70, 0x00, 10, 0xCC, 0x33, 0xC3, 0x3C}; //Background picture
//message ralated
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
//@fn: uart_1_input_proc / uart_3_input_proc
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

static rt_thread_t lcd_init = RT_NULL;
//@fn: lcd_init
//@brief: init the LCD screen
//
static void lcd_init_entry(void* parameter) {
  //TODO: display the picture 10
  //AA 70 <Pic_ID> CC 33 C3 3C
  unsigned int i =65535;
  rt_device_t device;
  rt_err_t result = RT_EOK;
  //configure the control pin, PB13 in this case
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  /* Configure PH2 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  device = rt_device_find("uart3");
  if (device == RT_NULL) {
    rt_kprintf("Device not found!\r\n");
  } else {
    result = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    rt_device_write(device, 0, &lcd_background_pic[0], sizeof(lcd_background_pic));
    while(i!=0){i--;}  //delay for a while or the data might not be sent before ctrl pin reset
    //DO NOT use rt_thread_delay to delay, or the thread will suspend
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
  }
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
  //init 2 message queue
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
                    "mq_uart3",  //message queue from uart1
                    &uart_3_msg_pool[0], //mem pool
                    128 - sizeof(void*),  //size of each message
                    sizeof(uart_3_msg_pool),  //sizeof uart_1_msg_pool
                    RT_IPC_FLAG_FIFO);  //First In First Out 
  if (result != RT_EOK) {   
    rt_kprintf("init message queue failed.\n");
  }                   
  
  //init the LCD
  //init thread processing data from zigbee
  rt_thread_init(&thread_uart_recv,
                 "uart_recv",
                 rt_thread_entry_uart_recv,
                 RT_NULL,
                 &thread_uart_recv_stack[0],
                 sizeof(thread_uart_recv_stack), 11, 10);
  rt_thread_startup(&thread_uart_recv); 
  
  //init thread processing data from upper machine
  rt_thread_init(&thread_upper_recv,
               "upper_recv",
               rt_thread_entry_upper_recv,
               RT_NULL,
               &thread_upper_recv_stack[0],
               sizeof(thread_upper_recv_stack), 11, 10);
  rt_thread_startup(&thread_upper_recv);
  
  //init lcd
  lcd_init = rt_thread_create("lcd_init", lcd_init_entry, (void*)1, 128, 10, 5);
  if (lcd_init != RT_NULL) {
    rt_thread_startup(lcd_init);
  }
  return 0;
}

/*@}*/
