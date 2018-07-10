#include <stdbool.h>
#include "control.h"
#include "usbd_custom_hid_if.h"


#define WHEEL_COUNT     4
#define LOOP_INTERVAL   10
#define LED_FLASH_FREQ  10
#define DIR_FORWARD         0x01
#define DIR_BACKWARD        0x02
#define DIR_BRAKE           0x00

#define FL_IDX             0
#define FR_IDX             1
#define TL_IDX             2
#define TR_IDX             3


typedef __packed struct
{
  uint32_t counters[WHEEL_COUNT];
  uint8_t throttles[WHEEL_COUNT];
  uint8_t direction;
  uint8_t dummy[11];
  uint8_t loopCount;
} State_t;

typedef __packed struct
{
  uint8_t throttles[WHEEL_COUNT];
  uint8_t direction;
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
  int targetThrottle;
  int currentThrottle;
  int brakeTime;
  int targetDirection;
  int currentDirection;
} Motor_t;


static Motor_t motors[] =
{
  {&htim2, TIM_CHANNEL_1, DRV_FL_A_GPIO_Port, DRV_FL_A_Pin, DRV_FL_B_GPIO_Port, DRV_FL_B_Pin, 1},
  {&htim4, TIM_CHANNEL_2, DRV_FR_A_GPIO_Port, DRV_FR_A_Pin, DRV_FR_B_GPIO_Port, DRV_FR_B_Pin, 1},
  {&htim1, TIM_CHANNEL_3, DRV_TL_A_GPIO_Port, DRV_TL_A_Pin, DRV_TL_B_GPIO_Port, DRV_TL_B_Pin, 1},
  {&htim2, TIM_CHANNEL_3, DRV_TR_A_GPIO_Port, DRV_TR_A_Pin, DRV_TR_B_GPIO_Port, DRV_TR_B_Pin, 1},
};

static State_t state;
static uint32_t tickStart;
static bool hasOutReportReceived;


static void USB_Connect(void)
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

static void LoopInit(void)
{
  int i;

  for(i = 0; i < WHEEL_COUNT; i++)
  {
    state.counters[i] = 0;
    state.throttles[i] = 0;
    motors[i].targetThrottle = 0;
    motors[i].currentThrottle = 0;
    motors[i].targetDirection = DIR_BRAKE;
    motors[i].currentDirection = DIR_BRAKE;
    motors[i].brakeTime = 0;
  }
  state.loopCount = 0;
  hasOutReportReceived = false;

  USBD_CustomHID_fops_FS.OutEvent = CUSTOM_HID_OutEvent_FS;
  USB_Connect();
}

static void Sonar(void)
{
  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_SET);

  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_RESET);
}

static void MotorUpdateTarget(uint8_t throttles[], uint8_t direction)
{
  int i = 0;
  while (i < WHEEL_COUNT) {
    motors[i].targetThrottle = throttles[i];
    motors[i].targetDirection = direction & 0x03;
    if (motors[i].targetDirection == DIR_BRAKE)
    {
      motors[i].targetThrottle = 0;
    }
    direction >>= 2;
    i++;
  }
}

static void MotorControl(void)
{
  int i, diff;
  Motor_t *drv;
  TIM_OC_InitTypeDef config;

  config.OCMode = TIM_OCMODE_PWM1;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  config.OCIdleState = TIM_OCIDLESTATE_RESET;
  config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  for (i = 0; i < WHEEL_COUNT; i++)
  {
    drv = &motors[i];
    if (drv->brakeTime != 0)
    {
      drv->brakeTime--;
    }
    // 不需要更新速度或者刹车未完成
    if ((drv->currentThrottle == drv->targetThrottle && drv->currentDirection == drv->targetDirection)
     || drv->brakeTime)
    {
      continue;
    }

    // 刹车
    if (drv->targetDirection == DIR_BRAKE ||
      (drv->currentDirection != drv->targetDirection && drv->currentThrottle && drv->targetThrottle))
    {
      HAL_GPIO_WritePin(drv->APort, drv->APin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(drv->BPort, drv->BPin, GPIO_PIN_RESET);
      config.Pulse = 100;
      drv->currentThrottle = 0;
      drv->currentDirection = DIR_BRAKE;
      // 刹车时间100ms
      drv->brakeTime = 100 / LOOP_INTERVAL;
    }
    else
    {
      // 控制方向
      if (drv->currentThrottle == 0)
      {
        if (drv->targetDirection == DIR_FORWARD)
        {
          // 前进
          HAL_GPIO_WritePin(drv->APort, drv->APin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(drv->BPort, drv->BPin, GPIO_PIN_SET);
        }
        else
        {
          // 后退
          HAL_GPIO_WritePin(drv->APort, drv->APin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(drv->BPort, drv->BPin, GPIO_PIN_RESET);
        }
        drv->currentDirection = drv->targetDirection;
      }
      if (drv->currentThrottle != drv->targetThrottle)
      {
        diff = drv->targetThrottle - drv->currentThrottle;
        if (diff > 7)
        {
          diff = 7;
        }
        else if (diff < -7)
        {
          diff = -7;
        }
        drv->currentThrottle += diff;
      }
      config.Pulse = drv->currentThrottle;
    }
    HAL_TIM_PWM_ConfigChannel(drv->htim, &config, drv->PWMChannel);
    HAL_TIM_PWM_Start(drv->htim, drv->PWMChannel);
  }
}

static void LedControl(uint8_t on)
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

static void ReportState(void)
{
  int i;
  uint8_t direction = 0;
  static uint8_t inReport[64];

  for (i = 0; i < WHEEL_COUNT; i++) {
    state.throttles[i] = motors[i].currentThrottle;
    direction <<= 2;
    direction |= motors[WHEEL_COUNT - i - 1].currentDirection;
  }
  state.direction = direction;
  memcpy(inReport, &state, sizeof(state));
  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, inReport, sizeof(inReport));
}

void MainLoop(void)
{
  ControlData_t *cData;

  LoopInit();

  tickStart = HAL_GetTick();

  while (1)
  {
    // frame control
    while (HAL_GetTick() - tickStart < LOOP_INTERVAL);
    tickStart += LOOP_INTERVAL;
    state.loopCount++;

    // LED2
    //if (state.loopCount % 10 == 0)
    {
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }

    // sonar
    if (state.loopCount % 5 == 0)
    {
      Sonar();
    }

    // control
    if (hasOutReportReceived)
    {
      hasOutReportReceived = false;
      cData = (ControlData_t *)((USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData)->Report_buf;
      MotorUpdateTarget(cData->throttles, cData->direction);
    }

    MotorControl();

    // report status
    ReportState();
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

