#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* GPIO mapping — change here if needed */
#define ENC_CLK_PIN   GPIO_PIN_0
#define ENC_CLK_PORT  GPIOE
#define ENC_DT_PIN    GPIO_PIN_5
#define ENC_DT_PORT   GPIOE
#define ENC_SW_PIN    GPIO_PIN_7
#define ENC_SW_PORT   GPIOB

/* Timing */
#define ENC_DEBOUNCE_MS     5
#define ENC_LONG_PRESS_MS   800
#define ENC_DOUBLE_PRESS_MS 300

typedef enum {
    ENC_EVENT_NONE = 0,
    ENC_EVENT_CW,           /* clockwise rotation */
    ENC_EVENT_CCW,          /* counter-clockwise rotation */
    ENC_EVENT_SHORT_PRESS,  /* select / enter edit */
    ENC_EVENT_LONG_PRESS,   /* back / save */
    ENC_EVENT_DOUBLE_PRESS, /* save to SD */
} EncoderEvent_t;

void Encoder_Init(void);
EncoderEvent_t Encoder_GetEvent(void);
void Encoder_EXTI_CLK_Callback(void);
void Encoder_EXTI_SW_Callback(void);

#endif