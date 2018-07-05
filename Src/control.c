#include "control.h"
#include "usbd_custom_hid_if.h"


#define WHEEL_COUNT     4
#define LOOP_INTERVAL   10
#define LED_FLASH_FREQ  10


typedef __packed struct
{
  uint8_t loopCount;
  uint32_t counters[WHEEL_COUNT];
  uint8_t throttles[WHEEL_COUNT];
} State_t;

typedef __packed struct
{
  uint8_t throttles[WHEEL_COUNT];
} Operation_t;


static State_t state;
static uint32_t tickStart;
static uint8_t report_buffer[64];


void USB_Connect(void)
{
  HAL_GPIO_WritePin(USB_CTL_GPIO_Port, USB_CTL_Pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(USB_CTL_GPIO_Port, USB_CTL_Pin, GPIO_PIN_SET);
}

static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  return (USBD_OK);
}

void Control_Init(void)
{
  int i = 0;
  
  for(i = 0; i < WHEEL_COUNT; i++)
  {
    state.counters[i] = 0;
    state.throttles[i] = 0;
  }
  state.loopCount = 0;
  
  USBD_CustomHID_fops_FS.OutEvent = CUSTOM_HID_OutEvent_FS;
  USB_Connect();
  HAL_TIM_PWM_Start(&htim_drv0, CHANNEL_DRV0);
  HAL_TIM_PWM_Start(&htim_drv1, CHANNEL_DRV1);
  HAL_TIM_PWM_Start(&htim_drv2, CHANNEL_DRV2);
  HAL_TIM_PWM_Start(&htim_drv3, CHANNEL_DRV3);
}

void Sonar(void)
{
  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_SET);
  
  //HAL_GPIO_WritePin(SONAR_TRIG_GPIO_Port, SONAR_TRIG_Pin, GPIO_PIN_RESET);
}


void Control_Loop(void)
{
  /*
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  */
  
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
    memcpy(report_buffer, &state, sizeof(state));
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report_buffer, 64);
    
    // control
    /*
    HAL_TIM_PWM_Stop(&htim_drv2, CHANNEL_DRV2);
    sConfigOC.Pulse += 10;
    if (sConfigOC.Pulse > htim_drv2.Init.Period)
    {  
      sConfigOC.Pulse = 0;
    }
    HAL_TIM_PWM_ConfigChannel(&htim_drv2, &sConfigOC, CHANNEL_DRV2);
    HAL_TIM_PWM_Start(&htim_drv2, CHANNEL_DRV2);
    */
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

