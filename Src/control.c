#include <stdbool.h>
#include "control.h"
#include "usbd_custom_hid_if.h"


#define WHEEL_COUNT     4
#define LOOP_INTERVAL   10
#define LED_FLASH_FREQ  10

#define FL_IDX             0
#define FR_IDX             1
#define TL_IDX             2
#define TR_IDX             3


typedef __packed struct
{
  uint32_t counters[WHEEL_COUNT];
  int8_t throttles[WHEEL_COUNT];
  int8_t dummy[12];
  uint8_t loopCount;
} State_t;

typedef __packed struct
{
  int8_t throttles[WHEEL_COUNT];
  uint8_t led;
} ControlData_t;

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t PWMChannel;
  GPIO_TypeDef *APort;
  uint32_t APin;
  GPIO_TypeDef *BPort;
  uint32_t BPin;
  uint32_t isBreak;
} Driver_t;


static Driver_t drvs[] =
{
  {&htim2, TIM_CHANNEL_1, DRV0_A_GPIO_Port, DRV0_A_Pin, DRV0_B_GPIO_Port, DRV0_B_Pin, 1},
  {&htim4, TIM_CHANNEL_2, DRV1_A_GPIO_Port, DRV1_A_Pin, DRV1_B_GPIO_Port, DRV1_B_Pin, 1},
  {&htim1, TIM_CHANNEL_3, DRV2_A_GPIO_Port, DRV2_A_Pin, DRV2_B_GPIO_Port, DRV2_B_Pin, 1},
  {&htim2, TIM_CHANNEL_3, DRV3_A_GPIO_Port, DRV3_A_Pin, DRV3_B_GPIO_Port, DRV3_B_Pin, 1},
};

static State_t state;
static uint32_t tickStart;
static uint8_t inReport[64];
static bool hasOutReportReceived;


void USB_Connect(void)
{
  HAL_GPIO_WritePin(USB_CTL_GPIO_Port, USB_CTL_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
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
    drvs[i].isBreak = 1;
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

void ThrottleControl(int8_t throttles[])
{
  int i;
  Driver_t *drv;
  TIM_OC_InitTypeDef config;

  config.OCMode = TIM_OCMODE_PWM1;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;

  for (i = 0; i < WHEEL_COUNT; i++)
  {
    drv = &drvs[i];
    // 需要控制方向
    if (((state.throttles[i] ^ throttles[i]) & 0x80) ||
          drv->isBreak)
    {
      drv->isBreak = 0;
      if (throttles[i] & 0x80)
      {
        HAL_GPIO_WritePin(drv->APort, drv->APin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(drv->BPort, drv->BPin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(drv->APort, drv->APin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(drv->BPort, drv->BPin, GPIO_PIN_SET);
      }
    }
    state.throttles[i] = throttles[i];
    config.Pulse = throttles[i] & 0x7F;
    HAL_TIM_PWM_Stop(drv->htim, drv->PWMChannel);
    HAL_TIM_PWM_ConfigChannel(drv->htim, &config, drv->PWMChannel);
    HAL_TIM_PWM_Start(drv->htim, drv->PWMChannel);
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
  //state.throttles[2] = 50;
  //ThrottleControl(state.throttles);
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
    case COUNTER_FL_Pin:
      state.counters[FL_IDX]++;
      break;
    case COUNTER_FR_Pin:
      state.counters[FR_IDX]++;
      break;
    case COUNTER_TL_Pin:
      state.counters[TL_IDX]++;
      break;
    case COUNTER_TR_Pin:
      state.counters[TR_IDX]++;
      break;
  }
}

