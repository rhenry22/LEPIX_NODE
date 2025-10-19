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

#include "chademo.h"
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


/* List of commands and handlers */

typedef int (*cmd_func_t)(char **args, int argc);

typedef struct {
    const char *name;
    cmd_func_t func;
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    { "reset", process_cmd_reset },
    { "flash", process_cmd_flash },
    { "power", app_process_cmd_power },
    { "evse", evse_process_cmd },
#ifdef ENABLE_CHADEMO
    { "chademo", chademo_process_cmd },
#endif
    { "hv", app_process_cmd_hv },
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
            process_stdin_line(&line_buf[0], line_buf_len);
            line_buf_len = 0;
            memset(line_buf, 0, APP_RX_DATA_SIZE);
        }
    }
}

bool cmd_init(void)
{
  taskHandle = osThreadNew(cmdTask, NULL, &taskAttributes);
  cmdMutexHandle = xSemaphoreCreateBinary();

  return (taskHandle != NULL && cmdMutexHandle != NULL);
}
