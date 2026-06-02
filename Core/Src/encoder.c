#include "encoder.h"

static volatile int8_t  enc_delta    = 0;
static volatile uint8_t sw_state     = 0;
static volatile uint32_t sw_press_t  = 0;
static volatile uint32_t sw_rel_t    = 0;
static volatile uint8_t  sw_pending  = 0;
static volatile uint8_t  last_clk    = 0;

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CLK — PE0 */
    GPIO_InitStruct.Pin  = ENC_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_CLK_PORT, &GPIO_InitStruct);

    /* DT — PE5 (input only, read during CLK IRQ) */
    GPIO_InitStruct.Pin  = ENC_DT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_DT_PORT, &GPIO_InitStruct);

    /* SW — PB7 */
    GPIO_InitStruct.Pin  = ENC_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_SW_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn,     5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn,   5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    last_clk = HAL_GPIO_ReadPin(ENC_CLK_PORT, ENC_CLK_PIN);
}

/* Called from EXTI0 IRQ handler (PE0 = CLK) */
void Encoder_EXTI_CLK_Callback(void)
{
    static uint32_t last_t = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_t < ENC_DEBOUNCE_MS) return;
    last_t = now;

    uint8_t clk = HAL_GPIO_ReadPin(ENC_CLK_PORT, ENC_CLK_PIN);
    uint8_t dt  = HAL_GPIO_ReadPin(ENC_DT_PORT,  ENC_DT_PIN);

    if (clk != last_clk) {
        if (clk == 0) {
            enc_delta += (dt != clk) ? +1 : -1;
        }
        last_clk = clk;
    }
}

/* Called from EXTI9_5 IRQ handler (PB7 = SW) */
void Encoder_EXTI_SW_Callback(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t sw   = HAL_GPIO_ReadPin(ENC_SW_PORT, ENC_SW_PIN);

    if (sw == GPIO_PIN_RESET) {
        /* Button pressed */
        sw_press_t = now;
        sw_state   = 1;
    } else {
        /* Button released */
        if (sw_state == 1) {
            sw_rel_t = now;
            sw_pending++;
        }
        sw_state = 0;
    }
}

EncoderEvent_t Encoder_GetEvent(void)
{
    /* Rotation */
    if (enc_delta != 0) {
        int8_t d = enc_delta;
        enc_delta = 0;
        return (d > 0) ? ENC_EVENT_CW : ENC_EVENT_CCW;
    }

    /* Long press (button held) */
    if (sw_state == 1) {
        if ((HAL_GetTick() - sw_press_t) >= ENC_LONG_PRESS_MS) {
            sw_state = 2; /* consumed */
            return ENC_EVENT_LONG_PRESS;
        }
    }

    /* Short / double press (after release) */
    if (sw_pending > 0) {
        uint32_t now = HAL_GetTick();
        if ((now - sw_rel_t) >= ENC_DOUBLE_PRESS_MS || sw_pending >= 2) {
            uint8_t count = sw_pending;
            sw_pending = 0;
            if (count >= 2) return ENC_EVENT_DOUBLE_PRESS;
            return ENC_EVENT_SHORT_PRESS;
        }
    }

    return ENC_EVENT_NONE;
}