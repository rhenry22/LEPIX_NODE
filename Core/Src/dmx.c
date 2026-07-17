#include "dmx.h"
#include "usart.h"    /* huart2 */
#include "main.h"     /* HAL, RS485_TX_RX__Pin, GPIOD */
#include <string.h>

/* ─── Paramètres de timing ─────────────────────────────────────────────── */
#define DMX_BAUD          250000u   /* débit DMX512 nominal            */
#define DMX_BREAK_BAUD     90000u    /* débit réduit pour générer le BREAK :
                                      * start+8 bits à 0 = 9 bits ~100 µs   */
#define DMX_REFRESH_MS        25u    /* période de rafraîchissement (40 Hz) */

/* Direction MAX485 : PD7 HIGH = émission, LOW = écoute (RDM plus tard). */
#define DMX_DIR_TX()  HAL_GPIO_WritePin(RS485_TX_RX__GPIO_Port, RS485_TX_RX__Pin, GPIO_PIN_SET)
#define DMX_DIR_RX()  HAL_GPIO_WritePin(RS485_TX_RX__GPIO_Port, RS485_TX_RX__Pin, GPIO_PIN_RESET)

/* DMA1_Stream6 Channel4 = USART2_TX (RM0090). */
static DMA_HandleTypeDef s_hdma_tx;

static void DMX_TxCpltHandler(void);   /* forward */

/* Buffer de trame : [0] = start code (0x00), [1..512] = slots DMX. */
static uint8_t  s_frame[1 + DMX_SLOTS];
static uint8_t  s_work[1 + DMX_SLOTS];   /* buffer d'écriture (double) */
static volatile bool s_dirty      = false;
static volatile bool s_tx_busy    = false;

/* Machine à états de la trame. */
typedef enum {
    DMX_ST_IDLE = 0,
    DMX_ST_BREAK,     /* octet 0x00 en cours d'envoi à baudrate réduit */
    DMX_ST_DATA,      /* 513 octets en cours d'envoi à 250k            */
} dmx_state_t;

static volatile dmx_state_t s_state = DMX_ST_IDLE;
static uint32_t s_last_refresh_ms = 0;
static uint8_t  s_break_byte = 0x00;

/* Reconfigure le baudrate de USART2 sans réinitialiser tout le périphérique. */
static void dmx_set_baud(uint32_t baud)
{
    huart2.Init.BaudRate = baud;
    /* Recalcule BRR à partir du nouveau baudrate. */
    huart2.Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud);
}

void DMX_Init(void)
{
    /* USART2 : 250000 baud, 8 bits, 2 stops, sans parité. */
    HAL_UART_DeInit(&huart2);
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = DMX_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_2;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* DMA1_Stream6 Channel4 = USART2_TX (mémoire -> périphérique). */
    __HAL_RCC_DMA1_CLK_ENABLE();
    s_hdma_tx.Instance                 = DMA1_Stream6;
    s_hdma_tx.Init.Channel             = DMA_CHANNEL_4;
    s_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_hdma_tx.Init.Mode                = DMA_NORMAL;
    s_hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&s_hdma_tx);
    __HAL_LINKDMA(&huart2, hdmatx, s_hdma_tx);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* Broche de direction MAX485 (PD7) en sortie, écoute par défaut. */
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gi.Pin   = RS485_TX_RX__Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_TX_RX__GPIO_Port, &gi);
    DMX_DIR_RX();

    /* Buffers à zéro, start code = 0x00. */
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_work,  0, sizeof(s_work));
    s_state = DMX_ST_IDLE;
}

void DMX_SetSlot(uint16_t idx, uint8_t value)
{
    if (idx < DMX_SLOTS)
        s_work[1 + idx] = value;
}

void DMX_SetSlots(uint16_t start_slot, const uint8_t *data, uint16_t count)
{
    if (start_slot >= DMX_SLOTS || data == NULL)
        return;
    if (start_slot + count > DMX_SLOTS)
        count = DMX_SLOTS - start_slot;
    memcpy(&s_work[1 + start_slot], data, count);
}

void DMX_Commit(void)
{
    s_dirty = true;
}

bool DMX_Busy(void)
{
    return s_tx_busy;
}

/* Lance l'émission d'une trame : BREAK (0x00 à baudrate réduit) d'abord. */
static void dmx_start_frame(void)
{
    /* Fige le contenu à émettre. */
    memcpy(s_frame, s_work, sizeof(s_frame));
    s_frame[0] = 0x00;   /* start code DMX (null) */

    s_tx_busy = true;
    DMX_DIR_TX();

    /* BREAK : un octet 0x00 à baudrate réduit -> ~100 µs de niveau bas. */
    s_state = DMX_ST_BREAK;
    dmx_set_baud(DMX_BREAK_BAUD);
    HAL_UART_Transmit_DMA(&huart2, &s_break_byte, 1);
}

/* Callback HAL de fin de transmission : on ne traite que USART2 (DMX). */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        DMX_TxCpltHandler();
}

/* Callback fin de transmission USART2 (BREAK terminé -> données ; ou fin). */
static void DMX_TxCpltHandler(void)
{
    if (s_state == DMX_ST_BREAK) {
        /* Le MARK-after-break (>=8 µs) est assuré par le stop bit + le
         * temps de reconfiguration ; on enchaîne sur les données à 250k. */
        s_state = DMX_ST_DATA;
        dmx_set_baud(DMX_BAUD);
        HAL_UART_Transmit_DMA(&huart2, s_frame, sizeof(s_frame));
    } else if (s_state == DMX_ST_DATA) {
        /* Trame complète émise. */
        s_state   = DMX_ST_IDLE;
        s_tx_busy = false;
        DMX_DIR_RX();   /* relâche le bus (utile pour RDM à l'étape 4) */
    }
}

/* À appeler depuis DMA1_Stream6_IRQHandler (USART2_TX). */
void DMX_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_tx);
}

void DMX_Task(void)
{
    if (s_state != DMX_ST_IDLE)
        return;   /* trame en cours */

    uint32_t now = HAL_GetTick();
    if ((now - s_last_refresh_ms) < DMX_REFRESH_MS)
        return;

    /* On émet à cadence fixe (les récepteurs DMX attendent un flux continu),
     * qu'il y ait eu changement ou non. s_dirty sert juste au monitoring. */
    s_last_refresh_ms = now;
    s_dirty = false;
    dmx_start_frame();
}
