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
  *
  * ---------------------------------------------------------------------------
  * MODIFIED BY FIXAPOSTURE AB
  * ---------------------------------------------------------------------------
  * Organisation:      Fixaposture AB
  * Modified by:       Goran Nybom  <gnybom@gmail.com>
  * Cortex extension:  GitHub Copilot
  * First modified:    2026-10-08
  * Project:           FixaSpine WB55 - BLE over UART
  * Reference:         BLE_IMPLEMENTATION_PLAN_2026-10-03.md
  *                    BLE_PHASE2_CODING_PLAN_2026-10-08.md
  *
  * Summary of changes: application behaviour changed from LED control /
  * button notification to a transparent byte-passthrough echo. Bytes written
  * to the NUS RX characteristic are buffered in the HCI event context and
  * echoed back as TX notifications from the existing FreeRTOS notify thread.
  * The Nucleo B1 button now sends a fixed test string.
  *
  * All modifications are confined to USER CODE regions.
  * See git history for per-change detail.
  * ---------------------------------------------------------------------------
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

/* PHASE 2 constants.
 *
 * These are deliberately placed HERE and not in the "Private defines" (PD)
 * USER CODE region further down: CubeMX emits the PD region *after* the
 * Custom_App_Context_t typedef, and EchoBuf[] inside that struct needs the
 * size at the point of declaration. Defining it in PD gives
 * "error: 'CUSTOM_APP_ECHO_BUF_SIZE' undeclared here (not in a function)".
 * The Includes region is the first USER CODE region in the file, so it is the
 * correct home for anything the typedefs depend on.
 */

/* Largest payload that fits one notification PDU at CFG_BLE_MAX_ATT_MTU = 156
 * (MTU minus 3 bytes of ATT overhead).
 * Must track CUSTOM_STM_MAX_PAYLOAD_LEN in custom_stm.c. */
#define CUSTOM_APP_ECHO_BUF_SIZE      153

/* Text sent when the Nucleo B1 button is pressed - lets us prove the TX path
 * without a phone attached. */
#define CUSTOM_APP_BUTTON_TEST_STRING "FixaSpine BLE TX test\r\n"

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

  /* PHASE 2 (BLE_PHASE2_CODING_PLAN_2026-10-08.md) - echo buffer.
   *
   * Hand-off discipline, because these two fields are touched from two
   * different contexts:
   *   WRITER : Custom_STM_App_Notification(), running in the HCI user-event
   *            thread context, fills EchoBuf then EchoLen, then signals.
   *   READER : Custom_Switch_c_Send_Notification() task, wakes on the flag and
   *            consumes EchoLen then EchoBuf.
   * Order is always write-payload -> write-length -> signal, so the reader can
   * never observe a length that is larger than the bytes actually present.
   * A single buffer is sufficient here: BLE write-without-response from one
   * central is not pipelined faster than the notify task drains it at this
   * MTU, and an overrun merely overwrites an un-echoed line rather than
   * corrupting one. A ring buffer is a Phase 3 concern, not a Phase 2 one.
   */
  uint8_t               EchoBuf[CUSTOM_APP_ECHO_BUF_SIZE];
  volatile uint8_t      EchoLen;

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* PHASE 2 constants live in the USER CODE BEGIN Includes region above, because
 * Custom_App_Context_t needs CUSTOM_APP_ECHO_BUF_SIZE and this region is
 * emitted after that typedef. */

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

      /* ====================================================================
       * PHASE 2 (item A1) - NUS RX: byte passthrough.
       *
       * Tom's demo decoded byte[1] as an LED command. That is replaced by a
       * transparent copy of whatever the central wrote, which is then echoed
       * back as a TX notification by Custom_Switch_c_Send_Notification().
       *
       * The echo is deliberately NOT sent from here. This function runs in the
       * HCI user-event context; calling aci_gatt_update_char_value() from
       * inside a stack event callback is the classic re-entrancy trap on
       * STM32WB. We hand off to the existing notify thread instead.
       * ==================================================================== */
      {
        uint8_t len = pNotification->DataTransfered.Length;

        APP_DBG_MSG("\r\n** NUS RX: %d byte(s)\n", len);

        if (len == 0)
        {
          APP_DBG_MSG("-- NUS RX: empty write, nothing to echo\n");
          break;
        }

        /* Defensive clamp. The characteristic is declared with a maximum of
         * CUSTOM_APP_ECHO_BUF_SIZE, so the stack should already enforce this,
         * but a silent truncation is far better than a buffer overrun. */
        if (len > CUSTOM_APP_ECHO_BUF_SIZE)
        {
          APP_DBG_MSG("-- NUS RX: %d bytes truncated to %d\n", len, CUSTOM_APP_ECHO_BUF_SIZE);
          len = CUSTOM_APP_ECHO_BUF_SIZE;
        }

        /* Order matters - payload, then length, then signal. See the comment
         * on EchoBuf/EchoLen in Custom_App_Context_t. */
        memcpy(Custom_App_Context.EchoBuf, pNotification->DataTransfered.pPayload, len);
        Custom_App_Context.EchoLen = len;

        if (Custom_Switch_c_Send_NotificationId != NULL)
        {
          osThreadFlagsSet(Custom_Switch_c_Send_NotificationId, 1);
        }
      }

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

  for(;;)
  {
     osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);     // wait on event, handle with this task thread

    /* USER CODE BEGIN Switch_c_NS_1*/

    /* ====================================================================
     * PHASE 2 (item A2) - NUS TX: echo the bytes captured by the RX handler.
     *
     * Tom's button-toggle payload and the ST BLE Toolbox "first byte must be
     * 0x01" workaround are both gone - they were artefacts of the P2P demo
     * protocol and have no meaning for a transparent UART pipe.
     * ==================================================================== */
    {
      uint8_t len = Custom_App_Context.EchoLen;

      Custom_App_Context.EchoLen = 0;   /* consume */

      if (len == 0)
      {
        /* Woken with nothing to send - e.g. a spurious signal. */
        APP_DBG_MSG("-- NUS TX: woken with empty buffer, nothing to send\n");
      }
      else if (!Custom_App_Context.Switch_c_Notification_Status)
      {
        /* The central has not subscribed to the TX characteristic. Sending
         * anyway would just be rejected by the stack. */
        APP_DBG_MSG("-- NUS TX: notifications DISABLED, dropping %d byte(s)\n", len);
      }
      else
      {
        tBleStatus ret;

        memcpy(NotifyCharData, Custom_App_Context.EchoBuf, len);

        /* Length-aware send. Custom_STM_App_Update_Char() would always push
         * SizeSwitch_C (153) bytes regardless of the real payload, which would
         * pad every echo with stale buffer contents. */
        ret = Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_SWITCH_C,
                                                         (uint8_t *)NotifyCharData,
                                                         len);

        if (ret == BLE_STATUS_SUCCESS)
        {
          APP_DBG_MSG("-- NUS TX: echoed %d byte(s)\n", len);
        }
        else
        {
          APP_DBG_MSG("-- NUS TX: echo FAILED, status 0x%02X, %d byte(s)\n", ret, len);
        }
      }
    }

    /* USER CODE END Switch_c_NS_1*/

    /* USER CODE BEGIN Switch_c_NS_Last*/

    /* USER CODE END Switch_c_NS_Last*/
  }
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

// ============================
// From custom service example: https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_STM32CubeMX
//
// PHASE 2 (item A3): repurposed. Instead of toggling a button state, pressing
// B1 queues a fixed test string on the TX characteristic. This proves the
// device -> phone direction on its own, which is useful when the RX path is
// the thing under suspicion.
//
// Called from interrupt context, so it only fills the buffer and signals.
void P2PS_APP_SW1_Button_Action(void)
{
  static const char test_string[] = CUSTOM_APP_BUTTON_TEST_STRING;
  uint8_t len = (uint8_t)(sizeof(test_string) - 1U);   /* drop the NUL */

  if (len > CUSTOM_APP_ECHO_BUF_SIZE)
  {
    len = CUSTOM_APP_ECHO_BUF_SIZE;
  }

  /* Payload, then length, then signal - same ordering rule as the RX path. */
  memcpy(Custom_App_Context.EchoBuf, test_string, len);
  Custom_App_Context.EchoLen = len;

  // NOTE: Re-implemented with FreeRTOS
  // UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
  if (Custom_Switch_c_Send_NotificationId != NULL)
  {
    osThreadFlagsSet(Custom_Switch_c_Send_NotificationId, 1);  // signal thread (can be from interrupt)
  }
  return;
}
// ============================

/* USER CODE END FD_LOCAL_FUNCTIONS*/
