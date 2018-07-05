#include <stdbool.h>
#include "control.h"
#include "usbd_custom_hid_if.h"


#define WHEEL_COUNT     4
#define LOOP_INTERVAL   10
#define LED_FLASH_FREQ  10

#define htim_drv0          htim2
#define htim_drv1          htim3
#define htim_drv2          htim1
#define htim_drv3          htim2
#define CHANNEL_DRV0       TIM_CHANNEL_1
#define CHANNEL_DRV1       TIM_CHANNEL_2
#define CHANNEL_DRV2       TIM_CHANNEL_1
#define CHANNEL_DRV3       TIM_CHANNEL_3



typedef __packed struct
{
  uint8_t loopCount;
  uint32_t counters[WHEEL_COUNT];
  uint8_t throttles[WHEEL_COUNT];
} State_t;

typedef __packed struct
{
  uint8_t throttles[WHEEL_COUNT];
  uint8_t led;
} ControlData_t;

static TIM_HandleTypeDef *phtims[] = {&htim2, &htim3, &htim1, &htim2};
static uint32_t drvChannels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3};
static State_t state;
static uint32_t tickStart;
static uint8_t inReport[64];
static bool hasOutReportReceived;


void USB_Connect(void)
{
  HAL_GPIO_WritePin(USB_CTL_GPIO_Port, USB_CTL_Pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(USB_CTL_GPIO_Port, USB_CTL_Pin, GPIO_PIN_SET);
}

static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  hasOutReportReceived = true;
  return (USBD_OK);
}

void Control_Init(void)
{
  int i;
  
  for(i = 0; i < WHEEL_COUNT; i++)
  {
    state.counters[i] = 0;
    state.throttles[i] = 0;
  }
  state.loopCount = 0;
  hasOutReportReceived = false;
  
  USBD_CustomHID_fops_FS.OutEvent = CUSTOM_HID_OutEvent_FS;
  USB_Connect();
}

void Sonar(void)
{
  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_SET);
  
  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_RESET);
}

void ThrottleControl(uint8_t throttles[])
{
  int i;
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  for (i = 0; i < WHEEL_COUNT; i++)
  {
    HAL_TIM_PWM_Stop(phtims[i], drvChannels[i]);
    sConfigOC.Pulse = throttles[i];
    HAL_TIM_PWM_ConfigChannel(phtims[i], &sConfigOC, drvChannels[i]);
    HAL_TIM_PWM_Start(phtims[i], drvChannels[i]);    
  }
}

void LedControl(uint8_t on)
{
  /* output low for led on */
  if (on)
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  }
}


void Control_Loop(void)
{
  ControlData_t *cData;
  
  tickStart = HAL_GetTick();
  while (1)
  {
    // frame control
    while (HAL_GetTick() - tickStart < LOOP_INTERVAL);
    tickStart += LOOP_INTERVAL;
    state.loopCount++;
    
    // LED2
    if (state.loopCount % 10 == 0)
    {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
    
    // sonar
    if (state.loopCount % 5 == 0)
    {
      Sonar();
    }
    
    // report status
    memcpy(inReport, &state, sizeof(state));
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, inReport, sizeof(inReport));
    
    // control
    if (hasOutReportReceived)
    {
      hasOutReportReceived = false;
      cData = (ControlData_t *)((USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData)->Report_buf;
      ThrottleControl(cData->throttles);
      //LedControl(cData->led);
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case COUNTER0_Pin:
      state.counters[0]++;
      break;
    case COUNTER1_Pin:
      state.counters[1]++;
      break;
    case COUNTER2_Pin:
      state.counters[2]++;
      break;
    case COUNTER3_Pin:
      state.counters[3]++;
      break;
  }
}

