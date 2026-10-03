/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
// #include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "cmsis_os2.h"

extern osThreadId_t Custom_Switch_c_Send_NotificationId;
extern const osThreadAttr_t Custom_Switch_c_Send_Notification_attr;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* My_P2P_Server */
  uint8_t               Switch_c_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  
  // ============================
  // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
  uint8_t               SW1_Status;                      /* Code Line to add */
  // ============================
  
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* My_P2P_Server */
static void Custom_Switch_c_Update_Char(void);
static void Custom_Switch_c_Send_Notification(void *argument);

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

    /* My_P2P_Server */
    case CUSTOM_STM_LED_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_LED_C_READ_EVT */
      break;

    case CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      
      // ============================
      // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT \n");
      APP_DBG_MSG("\r\n\r** Write Data: 0x%02X %02X \n", pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
      if(pNotification->DataTransfered.pPayload[1] == 0x01)
      {
        BSP_LED_On(LED_BLUE);
      }
      if(pNotification->DataTransfered.pPayload[1] == 0x00)
      {
        BSP_LED_Off(LED_BLUE); 
      } 
      // ============================

      /* USER CODE END CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT */
      
      // ============================
      // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_BUTTON_C_NOTIFY_ENABLED_EVT \n");
      Custom_App_Context.Switch_c_Notification_Status = 1;        /* My_Switch_Char notification status has been enabled */
      // ============================
      
      /* USER CODE END CUSTOM_STM_SWITCH_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT */
      
      // ============================
      // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_BUTTON_C_NOTIFY_DISABLED_EVT \n");
      Custom_App_Context.Switch_c_Notification_Status = 0;        /* My_Switch_Char notification status has been disabled */
      // ============================

      /* USER CODE END CUSTOM_STM_SWITCH_C_NOTIFY_DISABLED_EVT */
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
  
  // ============================
  // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX

  // UTIL_SEQ_RegTask(1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Custom_Switch_c_Send_Notification);
  
  // Re-implemented using FreeRTOS (following ST's model)
  Custom_Switch_c_Send_NotificationId = osThreadNew(Custom_Switch_c_Send_Notification, NULL, &Custom_Switch_c_Send_Notification_attr);
  Custom_App_Context.Switch_c_Notification_Status = 0;   
  Custom_App_Context.SW1_Status = 0;                 
  // ============================

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

/* My_P2P_Server */
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

void Custom_Switch_c_Send_Notification(void *argument) // task process
{
  UNUSED(argument);               // CMSIS compatibility
  uint8_t updateflag = 0;

  for(;;)
  {
     osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);     // wait on event, handle with this task thread

    /* USER CODE BEGIN Switch_c_NS_1*/
    // ============================
    // From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
    if (Custom_App_Context.Switch_c_Notification_Status)
    {
      updateflag = 1;

      // ST BLE Toolbox error - it uses the first byte to send the alarm (which should be Device_Button_Selection as ButtonPressed in BLE_p2pServer).
      // The app queries the wrong byte, but it works because the wrong byte (Device_Button_Selection) was always set to 1.
      // However, in the below code (using an array instead of a struct) they were not setting the first element to 1. The (hack) fix is
      // to do what is done in BLE_p2pServer by setting the first byte to 1. Now it works and "Button pressed" is displayed in the app:

      if (Custom_App_Context.SW1_Status == 0)
      {
        Custom_App_Context.SW1_Status = 1;
        // NotifyCharData[0] = 0x00;                                 
        NotifyCharData[0] = 0x01;     // (fix to match BLE_p2pServer)
        NotifyCharData[1] = 0x01;      
      }
      else
      {
        Custom_App_Context.SW1_Status = 0;
        // NotifyCharData[0] = 0x00;
        NotifyCharData[0] = 0x01;     // (fix to match BLE_p2pServer)
        NotifyCharData[1] = 0x00;
      }

      APP_DBG_MSG("-- CUSTOM APPLICATION SERVER: INFORM CLIENT BUTTON 1 PUSHED \n");
    }
    else
    {
      APP_DBG_MSG("-- CUSTOM APPLICATION: CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n");
    }     
    // ============================

    /* USER CODE END Switch_c_NS_1*/

    if (updateflag != 0)
    {
      Custom_STM_App_Update_Char(CUSTOM_STM_SWITCH_C, (uint8_t *)NotifyCharData);
    }

    /* USER CODE BEGIN Switch_c_NS_Last*/

    /* USER CODE END Switch_c_NS_Last*/
  }
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

// ============================
// From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
void P2PS_APP_SW1_Button_Action(void)
{
  // NOTE: Re-implemented with FreeRTOS
  // UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
  osThreadFlagsSet(Custom_Switch_c_Send_NotificationId, 1);                  // signal thread (can be from interrupt)
  return;
}
// ============================

/* USER CODE END FD_LOCAL_FUNCTIONS*/
