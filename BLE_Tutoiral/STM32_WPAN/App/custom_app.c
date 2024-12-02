/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* my_p2p_server */
  uint8_t               Switch_c_Notification_Status;
  /* my_mouse */
  uint8_t               Mouse_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t               SW1_Status;
  int8_t                x;
  int8_t                y;
  uint8_t               mouse_click;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* my_p2p_server */
static void Custom_Switch_c_Update_Char(void);
static void Custom_Switch_c_Send_Notification(void);
/* my_mouse */
static void Custom_Mouse_Update_Char(void);
static void Custom_Mouse_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* my_p2p_server */
    case CUSTOM_STM_LED_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_LED_C_READ_EVT */
      break;

    case CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT \n");
      APP_DBG_MSG("\r\n\r** Write Data: 0x%02X %02X \n", pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
      if(pNotification->DataTransfered.pPayload[1] == 0x01)
      {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      }
      if(pNotification->DataTransfered.pPayload[1] == 0x00)
      {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      }
      /* USER CODE END CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_BUTTON_C_NOTIFY_ENABLED_EVT \n");

      Custom_App_Context.Switch_c_Notification_Status = 1;        /* My_Switch_Char notification status has been enabled */
      /* USER CODE END CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_BUTTON_C_NOTIFY_DISABLED_EVT \n");

      Custom_App_Context.Switch_c_Notification_Status = 0;        /* My_Switch_Char notification status has been disabled */
      /* USER CODE END CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT */
      break;

    /* my_mouse */
    case CUSTOM_STM_MOUSE_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOUSE_NOTIFY_ENABLED_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_MOUSE_NOTIFY_ENABLED_EVT \n");

      Custom_App_Context.Mouse_Notification_Status = 1;        /* mouse data notification status has been enabled */
      /* USER CODE END CUSTOM_STM_MOUSE_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MOUSE_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOUSE_NOTIFY_DISABLED_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_MOUSE_NOTIFY_DISABLED_EVT \n");

      Custom_App_Context.Mouse_Notification_Status = 0;        /* mouse data notification status has been enabled */
      /* USER CODE END CUSTOM_STM_MOUSE_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  UTIL_SEQ_RegTask(1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Custom_Switch_c_Send_Notification);
  UTIL_SEQ_RegTask(1<< CFG_TASK_MOUSE_DATA_SEND_ID, UTIL_SEQ_RFU, Custom_Mouse_Send_Notification);

  Custom_App_Context.Switch_c_Notification_Status = 0;
  Custom_App_Context.SW1_Status = 0;

  Custom_App_Context.Mouse_Notification_Status = 0;
  Custom_App_Context.x = 0x2; // x
  Custom_App_Context.y = 0x5; // y
  Custom_App_Context.mouse_click = 0x1; // mouse click

  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* my_p2p_server */
__USED void Custom_Switch_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Switch_c_UC_1*/

  /* USER CODE END Switch_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SWITCH_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Switch_c_UC_Last*/

  /* USER CODE END Switch_c_UC_Last*/
  return;
}

void Custom_Switch_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Switch_c_NS_1*/

  /* USER CODE END Switch_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SWITCH_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Switch_c_NS_Last*/
  if(Custom_App_Context.Switch_c_Notification_Status)
    {
      Custom_STM_App_Update_Char(CUSTOM_STM_SWITCH_C, (uint8_t *)NotifyCharData);

      if (Custom_App_Context.SW1_Status == 0)
      {
        Custom_App_Context.SW1_Status = 1;
        NotifyCharData[0] = 0x00;
        NotifyCharData[1] = 0x01;
      }
      else
      {
        Custom_App_Context.SW1_Status = 0;
        NotifyCharData[0] = 0x00;
        NotifyCharData[1] = 0x00;
      }

      APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n");
      APP_DBG_MSG(" \n\r");
      Custom_STM_App_Update_Char(CUSTOM_STM_SWITCH_C, (uint8_t *)NotifyCharData);

    }
    else
    {
      APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n");
    }
  /* USER CODE END Switch_c_NS_Last*/

  return;
}

/* my_mouse */
__USED void Custom_Mouse_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mouse_UC_1*/

  /* USER CODE END Mouse_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MOUSE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Mouse_UC_Last*/

  /* USER CODE END Mouse_UC_Last*/
  return;
}

void Custom_Mouse_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mouse_NS_1*/

  /* USER CODE END Mouse_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MOUSE, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Mouse_NS_Last*/
  if(Custom_App_Context.Mouse_Notification_Status)
    {
      Custom_STM_App_Update_Char(CUSTOM_STM_MOUSE, (uint8_t *)NotifyCharData);

      NotifyCharData[0] = Custom_App_Context.x;
      NotifyCharData[1] = Custom_App_Context.y;
      NotifyCharData[2] = Custom_App_Context.mouse_click;

      Custom_App_Context.x ++;
      Custom_App_Context.y ++;

      APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT MOUSE DATA UPDATED \n");
      APP_DBG_MSG(" \n\r");
      Custom_STM_App_Update_Char(CUSTOM_STM_MOUSE, (uint8_t *)NotifyCharData);

    }
    else
    {
      APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n");
    }
  /* USER CODE END Mouse_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_APP_SW1_Button_Action(void){
  UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void MOUSE_APP_Send_Data_Action(void){
  UTIL_SEQ_SetTask(1<<CFG_TASK_MOUSE_DATA_SEND_ID, CFG_SCH_PRIO_0);

  return;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
