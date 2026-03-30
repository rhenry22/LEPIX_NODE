/** @file cmd.c
 *  @brief Serial command processing
 *
 *  This contains functions and logic to process
 *  a string received from the serial interfaces
 *  and call the appropriate command handler.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usbd_cdc_if.h"

#include "main.h"
#include "chademo.h"
#include "solax.h"
#include "evse.h"

#define MAX_CMD_ARGS        (8)             /* Maximum number of command arguments */

static osThreadId_t taskHandle;
static const osThreadAttr_t taskAttributes = {
  .name = "cmdTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
static SemaphoreHandle_t cmdMutexHandle;


static uint8_t cmd_buf[APP_RX_DATA_SIZE];   /* Buffer for stdin commands */
static uint16_t cmd_buf_len = 0;            /* Length of stdin buffer */

static uint8_t line_buf[APP_RX_DATA_SIZE];  /* Buffer for command line */
static uint16_t line_buf_len;               /* Length of command buffer */

static bool init = false;

/**
  * @brief  Process Reset command
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
static int process_cmd_reset(char **args, int argc)
{
  HAL_NVIC_SystemReset();
  return 0;
}

/**
  * @brief  Process Flash command
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
static int process_cmd_flash(char **args, int argc)
{
  JumpToBootloader();
  return 0;
}

/**
  * @brief  Process ESP Programming (USB Serial Bridge)
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Does not return.
  */
static int process_cmd_esp(char **args, int argc)
{
  esp_prog_key = ESP_MODE_KEY;
  HAL_NVIC_SystemReset();
  return 0;
}


/* List of commands and handlers */

typedef int (*cmd_func_t)(char **args, int argc);

typedef struct {
    const char *name;
    cmd_func_t func;
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    { "reset", process_cmd_reset },
    { "flash", process_cmd_flash },
    { "espbridge", process_cmd_esp },
    { "power", app_process_cmd_power },
    { "evse", evse_process_cmd },
#ifdef TARGET_CHADEMO
    { "chademo", chademo_process_cmd },
#endif
    { "solax", solax_process_cmd },
    { "hv", app_process_cmd_hv },
    { "leds", app_process_cmd_leds },
};

/**
  * @brief  Process a line of stdin data
  * @param  ptr Pointer to received data
  * @param  len length of data
  * @retval None
  */
static void process_stdin_line(uint8_t *ptr, uint16_t len)
{
  char *args[MAX_CMD_ARGS];
  int argc = 0;

  /* Tokenize the command input */
  char *tok = strtok((char*)ptr, " \r\n");
  while (tok && argc < MAX_CMD_ARGS)
  {
      args[argc++] = tok;
      tok = strtok(NULL, " \r\n");
  }
  if (argc == 0) return;

  // Find and call handler
  for (size_t i = 0; i < sizeof(cmd_table)/sizeof(cmd_table[0]); ++i)
  {
      if (strcmp(args[0], cmd_table[i].name) == 0)
      {
          int ret = cmd_table[i].func(&args[1], argc - 1);
          printf("{\"controller\":[{\"cmd\":\"%s\",\"status\":\"%d\"}]}\n", args[0], ret);
          return;
      }
  }

  /* Unknown command */
  printf("{\"controller\":[{\"status\":-1,\"message\":\"Unknown command: %s\"}]}\n", args[0]);

}

/**
  * @brief  Process data received on stdin
  * @param  ptr Pointer to received data
  * @param  len length of data
  * @retval None
  */
void stdio_parser(uint8_t *ptr, uint16_t len)
{
  uint32_t i;

  if (!init)
    return;

  if (len < (APP_RX_DATA_SIZE - cmd_buf_len))
  {
    uint8_t *c = &cmd_buf[cmd_buf_len];

    memcpy(&cmd_buf[cmd_buf_len], ptr, len);
    cmd_buf_len += len;

    for (i=0; i<len; ++i, ++c)
    {
      if (*c == '\n' || *c == '\r')
      {
        uint32_t offset = cmd_buf_len - len + i;

        /* Ensure we're null terminated */
        cmd_buf[offset] = 0;

        /* The input was likely received from IRQ context */
        /* Process in our own task */
        memcpy(&line_buf[0], &cmd_buf[0], offset);
        line_buf_len = offset;

        cmd_buf_len = 0;
        memset(cmd_buf, 0, APP_RX_DATA_SIZE);

        if (xPortIsInsideInterrupt())
        {
            BaseType_t pxHigherPriorityTaskWoken;
            xSemaphoreGiveFromISR(cmdMutexHandle, &pxHigherPriorityTaskWoken);
        }
        else
        {
            xSemaphoreGive(cmdMutexHandle);
        }
      }
    }
  }
  else
  {
    /* Buffer overflow */
    cmd_buf_len = 0;
  }
}

/**
  * @brief  Process commands from stdin
  * @param  argument: Not used
  * @retval None
  */
void cmdTask(void *argument)
{
    while (1)
    {
        xSemaphoreTake(cmdMutexHandle, 1000);

        if (line_buf_len > 0)
        {
            /* Process the command */
            comm_session(true);
            process_stdin_line(&line_buf[0], line_buf_len);
            comm_session(false);
            line_buf_len = 0;
            memset(line_buf, 0, APP_RX_DATA_SIZE);
        }
    }
}

bool cmd_init(void)
{
  taskHandle = osThreadNew(cmdTask, NULL, &taskAttributes);
  cmdMutexHandle = xSemaphoreCreateBinary();

  init = (taskHandle != NULL && cmdMutexHandle != NULL);

  return init;
}


/* Command Handlers (Application Level) */

/**
  * @brief  Process Power Setting command
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int app_process_cmd_power(char **args, int argc)
{
  if (argc == 1)
  {
    power_offset = strtol(args[0], NULL, 10);
    solax_set_output_power(power_offset);
    return 0;
  }
  return -1;
}

int app_process_cmd_leds(char **args, int argc)
{
  int ret = 0;
  if (argc >= 3 && 0 == strcmp(args[0], "debug"))
  {
    int16_t mask = strtol(args[1], NULL, 16);
    int16_t val = strtol(args[2], NULL, 16);

    /* Only modify bits covered by mask */
    debug_leds &= ~mask;
    debug_leds |= (val & mask);
    /* Optional 4th arg: "flash" -> add any bits set to 1 (mask & val) to flash list. */
    if (argc >= 4 && 0 == strcmp(args[3], "flash"))
    {
      /* Add bits where mask says and val is 1 */
      flash_debug_leds |= (mask & val);
      /* Remove any bits from flash list where mask requested clearing (mask & ~val) */
      flash_debug_leds &= ~(mask & ~val);
    }
    else
    {
      /* No explicit "flash" arg -> clear flashing for the bits covered by mask */
      flash_debug_leds &= ~((uint16_t)mask);
    }
  }
  else if (argc >= 3 && 0 == strcmp(args[0], "user"))
  {
    int16_t mask = strtol(args[1], NULL, 16);
    int16_t val = strtol(args[2], NULL, 16);

    /* Update base user LED values */
    user_led_base &= ~mask;
    user_led_base |= (val & mask);

    /* Optional flash parameter: add/remove flashing for the bits being set */
    if (argc >= 4 && 0 == strcmp(args[3], "flash"))
    {
      /* Add bits where mask says and val is 1 */
      flash_user_mask |= (mask & val);
      /* Remove any bits from flash list where mask requested clearing (mask & ~val) */
      flash_user_mask &= ~(mask & ~val);
    }
    else
    {
      /* No explicit "flash" arg -> clear flashing for the bits covered by mask */
      flash_user_mask &= ~mask;
    }

    /* Immediately apply the current visible state for user LEDs (honour flash_state)
       If a user LED is flashing, show flash_state, otherwise show base value */
    if (flash_user_mask & 0x01)
      HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
    else
      HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (user_led_base & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (flash_user_mask & 0x02)
      HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
    else
      HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (user_led_base & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    ret = -1;
  }

  return ret;
}


/**
  * @brief  Process HV Test commands
  * @param  args Command arguments
  * @param  argc Number of arguments
  * @retval Status (0 = OK, -1 = Error / Unknown Command)
  */
int app_process_cmd_hv(char **args, int argc)
{
  int ret = -1;
  if (argc >= 2 && 0 == strcmp(args[0], "iso"))
  {
    int32_t tgt = strtol(args[1], NULL, 10);
    if (tgt < 0) tgt = 0;

    if (tgt == 0)
    {
      hv_iso_test_enable(false, 0);
      ret = 0;
    }
    else if (tgt >= HV_GEN_MIN_VOLTAGE && tgt <= HV_GEN_MAX_VOLTAGE)
    {
      hv_iso_test_enable(true, tgt);
      ret = 0;
    }
    else
    {
      /* Out of range */
      printf("{\"controller\":[{\"status\":-1,\"message\":\"HV target out of range (%d-%dV)\"}]}\n",
             HV_GEN_MIN_VOLTAGE, HV_GEN_MAX_VOLTAGE);
    }
  }
  else if (argc >= 1 && 0 == strcmp(args[0], "get"))
  {
    trigger_json_update();
    ret = 0;
  }
  return ret;
}
