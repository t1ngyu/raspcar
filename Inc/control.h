#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "usbd_customhid.h"

extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define htim_drv0          htim2
#define htim_drv1          htim3
#define htim_drv2          htim1
#define htim_drv3          htim2
#define CHANNEL_DRV0       TIM_CHANNEL_1
#define CHANNEL_DRV1       TIM_CHANNEL_2
#define CHANNEL_DRV2       TIM_CHANNEL_1
#define CHANNEL_DRV3       TIM_CHANNEL_3

void USB_Connect(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Control_Init(void);
void Control_Loop(void);



#endif // __CONTROL_H__
