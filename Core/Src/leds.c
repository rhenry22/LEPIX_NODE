/** @file leds.c
 *  @brief Functions to control debug and status LEDs
 *
 *  Copyright (c) 2026 ARTaylor.co.uk.
 *  All rights reserved.
 *
 *  @author Richard Taylor <richard@artaylor.co.uk>
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdlib.h>

#include "ioexp.h"

/* How often to toggle flashing LEDs (ms) */
#define FLASH_TOGGLE_TIME        (500)

static uint16_t debug_leds = 0;       /* Combined state of debug leds */
static uint16_t flash_debug_leds = 0; /* Bits for debug LEDs that should flash */
static uint8_t flash_user_mask = 0;   /* Bits (0x01,0x02) for user GPIOs that should flash */
static uint8_t user_led_base = 0;     /* Base values for user LEDs (bit0 -> GPIO1, bit1 -> GPIO2) */
static bool flash_state = false;      /* Current on/off state for flashed LEDs */
static uint32_t last_flash_toggle = 0;/* Last tick when flash_state toggled */

static osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

static void ledTaskEntry(void *argument);

bool leds_init(void)
{
  bool error = false;

  /* Initialise Top IO expander and clear LEDs */
  if (ioexp_init(IOEXP_TOP_LEDS))
  {
    ioexp_set_direction(IOEXP_TOP_LEDS, 0xFFFF);
    ioexp_set_output(IOEXP_TOP_LEDS, 0x0000);
  }
  else
  {
    error = true;
  }

  /* Initialise Bottom LED IO expander and clear LEDs */
  if (ioexp_init(IOEXP_BOT_LEDS))
  {
    ioexp_set_direction(IOEXP_BOT_LEDS, 0xFFFF);
    ioexp_set_output(IOEXP_BOT_LEDS, 0x0000);
  }
  else
  {
    error = true;
  }

  if (!error)
    ledTaskHandle = osThreadNew(ledTaskEntry, NULL, &ledTask_attributes);

  return (!error && ledTaskHandle != NULL);
}

void leds_set(uint16_t leds)
{
  debug_leds |= leds;
}

void leds_clear(uint16_t leds)
{
  debug_leds &= ~leds;
}

/**
  * @brief  Function implementing the ledTask thread.
  * @param  argument: Not used
  * @retval None
  */
void ledTaskEntry(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    /* Leds (Top 2 rows) + flashing support */
    {
      static uint16_t old_display_leds = 0;

      /* Handle flash toggle timing */
      if (flash_debug_leds != 0 || flash_user_mask != 0)
      {
        uint32_t now = HAL_GetTick();
        if (last_flash_toggle == 0 || (now - last_flash_toggle) >= FLASH_TOGGLE_TIME)
        {
          flash_state = !flash_state;
          last_flash_toggle = now;
        }
      }

      /* Compute the LEDs to display on the top IO expander, applying flashing */
      uint16_t display_leds = debug_leds;
      if (flash_debug_leds != 0 && !flash_state)
      {
        /* When flash_state is false, clear the flashing bits so they appear off */
        display_leds &= ~flash_debug_leds;
      }

      if (display_leds != old_display_leds)
      {
        old_display_leds = display_leds;
        ioexp_set_direction(IOEXP_TOP_LEDS, ~display_leds);
      }

      /* Update user GPIO LEDs (GPIO1 / GPIO2) according to flash state */
      /* If a user LED is marked for flashing, show flash_state, otherwise show base value */
      if (flash_user_mask & 0x01)
      {
        HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
      }
      else
      {
        HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, (user_led_base & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      if (flash_user_mask & 0x02)
      {
        HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (flash_state ? GPIO_PIN_SET : GPIO_PIN_RESET));
      }
      else
      {
        HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, (user_led_base & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

    osDelay(100);
  }
}

int leds_process_cmd(char **args, int argc)
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
