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
//background [AA][70][00 0A][CC 33 C3 3C]
//float number: AA 1A 01 <X> <Y> <Mode> <Fcolor> <Bcolor> <Int_Value> CC 33 C3 3C
//[AA][1A 02]
//[00 32 
//00 3C 
//93 33 
//F8 00 00 1F 42 F6 E6 66][CC 33 C3 3C]
static char lcd_background_pic[]={0xAA, 0x70, 0x00, 10, 0xCC, 0x33, 0xC3, 0x3C}; //Background picture
static char lcd_display_temp[] = {0xAA, 0x1A, 0x02, 
                                  0x00, 0x20, 0x00, 0x30, //pos
                                  0x32, 0xF3, //mode
                                  0x00, 0x1F, //Fcolor
                                  0xF8, 0x00, //Bcolor
                                  0x00, 0x00, 0x00, 0x00, // saved
                                  0xCC, 0x33, 0xC3, 0x3C};
//message ralated
static struct rt_messagequeue uart_1_rx_mq;  //message queue for uart1
static struct rt_messagequeue uart_3_rx_mq;  //message queue for uart3
static struct rt_messagequeue lcd_cmd_mq;   //message queue for lcd
static char uart_1_msg_pool[512]; //message pool, size 512
static char uart_3_msg_pool[512]; //message pool, size 512
static char lcd_cmd_msg_pool[64]; 

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

//@fn rt_str_reverse
//@brief message received is reversed, so we need to reverse again using this function
//@para *str the first byte in a string that needs reversed
//@para length the total length that needs to be reversed, 0 included
//e.g a = {1,2,3,4}, then rt_str_reverse(&a[0], 3) result in {4, 3, 2, 1}
static void rt_str_reverse(char *str, int length) {
  char temp, *end_ptr;
  end_ptr = str + length;
  while(end_ptr>str) {
    temp = *str;
    *str = *end_ptr;
    *end_ptr = temp;
    str++;
    end_ptr--;
  }
}


static rt_thread_t lcd_init = RT_NULL;
//@fn: lcd_init
//@brief: init the LCD screen with a background pic
//
static void lcd_init_entry(void* parameter) {
  //AA 70 <Pic_ID> CC 33 C3 3C where Pic_ID is 2 bytes
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
static void thread_entry_uart_recv(void* parameter) {
  float temp = 123.45;
  char temp_data[4];
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
      rt_memcpy(&temp_data[0], &temp, 4);
      //rt_device_write(device, 0, &temp_data[0], 4);
      rt_mq_send(&lcd_cmd_mq, &temp_data[0], 4);
      rt_device_write(device, 0, &uart_rx_buffer[0], rx_length);
      //rt_thread_delay(5);
    }
  }
}

static char thread_upper_recv_stack[512];
struct rt_thread thread_upper_recv;  //receive cmd from upper machine through uart3
static void thread_entry_upper_recv(void* parameter) {
  rt_device_t device;
  rt_err_t result = RT_EOK;
  struct rx_msg msg;
  static char uart_rx_buffer[32];
  device = rt_device_find("uart2");
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
      rt_thread_delay(5);
    }
  }
}

ALIGN(RT_ALIGN_SIZE)
static char thread_lcd_cmd_stack[512];
struct rt_thread thread_lcd_cmd;  //receive temp data from zigbee through uart1
static void thread_entry_lcd_cmd(void* parameter) {
  //TODO: complete lcd cmds
  rt_uint16_t length = 0;
  rt_device_t device;
  rt_err_t result = RT_EOK;
  static char lcd_buffer[8];
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
  }
  while (1)
  {
    result = rt_mq_recv(&lcd_cmd_mq, &lcd_buffer[0], 4, 50);
    
    if (result == -RT_ETIMEOUT) {
      //no message, meaning that no cmds
    }
    else if (result == RT_EOK) {
      length = lcd_cmd_mq.msg_size;
      rt_memcpy(&lcd_display_temp[13], &lcd_buffer[0], 4);
      rt_str_reverse(&lcd_display_temp[13], 3);
      GPIO_SetBits(GPIOB, GPIO_Pin_13);
      rt_device_write(device, 0, &lcd_display_temp[0], 21);
      while(length != 10000){length++;}
      GPIO_ResetBits(GPIOB, GPIO_Pin_13);
      rt_thread_delay(20);
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
                      128 - sizeof(void*),  //max size of each message
                      sizeof(uart_1_msg_pool),  //sizeof uart_1_msg_pool
                      RT_IPC_FLAG_FIFO);  //First In First Out 
  if (result != RT_EOK) {   
    rt_kprintf("init uart1 message queue failed.\n");
  } 
  result = rt_mq_init(&uart_3_rx_mq,  //message handler
                    "mq_uart3",  //message queue from uart1
                    &uart_3_msg_pool[0], //mem pool
                    128 - sizeof(void*),  //max size of each message
                    sizeof(uart_3_msg_pool),  //sizeof uart_1_msg_pool
                    RT_IPC_FLAG_FIFO);  //First In First Out 
  if (result != RT_EOK) {   
    rt_kprintf("init uart3 message queue failed.\n");
  }                   
  
  result = rt_mq_init(&lcd_cmd_mq,
                      "mq_lcd",
                      &lcd_cmd_msg_pool[0],
                      16,//128 - sizeof(void*),
                      sizeof(lcd_cmd_msg_pool),
                      RT_IPC_FLAG_FIFO);
  if (result != RT_EOK) {   
    rt_kprintf("init lcd message queue failed.\n");
  }                      
  //init the LCD
  //init thread processing data from zigbee
  rt_thread_init(&thread_uart_recv,
                 "uart_recv",
                 thread_entry_uart_recv,
                 RT_NULL,
                 &thread_uart_recv_stack[0],
                 sizeof(thread_uart_recv_stack), 11, 10);
  rt_thread_startup(&thread_uart_recv); 
  
  //init thread processing data from upper machine
  rt_thread_init(&thread_upper_recv,
               "upper_recv",
               thread_entry_upper_recv,
               RT_NULL,
               &thread_upper_recv_stack[0],
               sizeof(thread_upper_recv_stack), 12, 10);
  rt_thread_startup(&thread_upper_recv);
  
  rt_thread_init(&thread_lcd_cmd,
               "lcd_cmd",
               thread_entry_lcd_cmd,
               RT_NULL,
               &thread_lcd_cmd_stack[0],
               sizeof(thread_lcd_cmd_stack), 13, 10);
  rt_thread_startup(&thread_lcd_cmd);
  
  //init lcd
  lcd_init = rt_thread_create("lcd_init", lcd_init_entry, (void*)1, 128, 10, 5);
  if (lcd_init != RT_NULL) {
    rt_thread_startup(lcd_init);
  }
  return 0;
}

/*@}*/
