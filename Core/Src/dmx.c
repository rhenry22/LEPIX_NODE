#include "dmx.h"
#include "usart.h"    /* huart2 */
#include "main.h"     /* HAL, RS485_TX_RX__Pin, P5_GPIOD10_Pin, ... */
#include <string.h>

/* ─── Paramètres de timing ─────────────────────────────────────────────── */
#define DMX_BAUD          250000u   /* débit DMX512 nominal                 */
#define DMX_BREAK_BAUD     90000u    /* débit réduit pour le BREAK (~100 µs) */
#define DMX_REFRESH_MS        25u    /* période de rafraîchissement (40 Hz)  */

/* Machine à états d'un port. */
typedef enum {
    DMX_ST_IDLE = 0,
    DMX_ST_BREAK,     /* octet 0x00 en cours d'envoi à baudrate réduit */
    DMX_ST_DATA,      /* 513 octets en cours d'envoi à 250k            */
} dmx_state_t;

/* Un port DMX = 1 USART + 1 DMA TX + 1 broche de direction MAX485. */
typedef struct {
    USART_TypeDef      *usart;        /* instance (USART2 / USART3)      */
    UART_HandleTypeDef *huart;        /* handle HAL                      */
    DMA_HandleTypeDef   hdma_tx;      /* DMA TX dédié                    */
    /* Direction MAX485 (DE+/RE) */
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;

    uint8_t  frame[1 + DMX_SLOTS];    /* start code + 512 slots (émis)   */
    uint8_t  work [1 + DMX_SLOTS];    /* buffer d'écriture               */
    volatile dmx_state_t state;
    volatile bool        tx_busy;
    uint32_t last_refresh_ms;
} dmx_port_t;

static dmx_port_t s_ports[DMX_NUM_PORTS];
static uint8_t    s_break_byte = 0x00;

/* huart3 est déclaré ici (USART3 n'est pas géré par CubeMX dans ce projet). */
UART_HandleTypeDef huart3;

#define DIR_TX(p)  HAL_GPIO_WritePin((p)->dir_port, (p)->dir_pin, GPIO_PIN_SET)
#define DIR_RX(p)  HAL_GPIO_WritePin((p)->dir_port, (p)->dir_pin, GPIO_PIN_RESET)

/* Reconfigure le baudrate sans réinitialiser tout le périphérique. */
static void port_set_baud(dmx_port_t *p, uint32_t baud)
{
    p->huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud);
}

/* Configure un USART en DMX (250k 8N2) + son DMA TX + sa broche direction. */
static void port_init(dmx_port_t *p,
                      USART_TypeDef *usart, UART_HandleTypeDef *huart,
                      DMA_Stream_TypeDef *dma_stream, uint32_t dma_channel,
                      GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    p->usart    = usart;
    p->huart    = huart;
    p->dir_port = dir_port;
    p->dir_pin  = dir_pin;

    huart->Instance          = usart;
    huart->Init.BaudRate     = DMX_BAUD;
    huart->Init.WordLength   = UART_WORDLENGTH_8B;
    huart->Init.StopBits     = UART_STOPBITS_2;
    huart->Init.Parity       = UART_PARITY_NONE;
    huart->Init.Mode         = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);

    /* DMA TX (mémoire -> périphérique). */
    p->hdma_tx.Instance                 = dma_stream;
    p->hdma_tx.Init.Channel             = dma_channel;
    p->hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    p->hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    p->hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    p->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    p->hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    p->hdma_tx.Init.Mode                = DMA_NORMAL;
    p->hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    p->hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&p->hdma_tx);
    __HAL_LINKDMA(huart, hdmatx, p->hdma_tx);

    /* Broche de direction MAX485 en sortie, écoute par défaut. */
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = dir_pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(dir_port, &gi);
    DIR_RX(p);

    memset(p->frame, 0, sizeof(p->frame));
    memset(p->work,  0, sizeof(p->work));
    p->state           = DMX_ST_IDLE;
    p->tx_busy         = false;
    p->last_refresh_ms = 0;
}

/* GPIO AF pour USART3 sur PC10 (TX) / PC11 (RX) — pas géré par le MSP CubeMX. */
static void usart3_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gi.Pin       = GPIO_PIN_10 | GPIO_PIN_11;   /* PC10=TX, PC11=RX */
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &gi);
}

void DMX_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* USART3 : le MSP CubeMX ne le configure pas -> GPIO AF à la main. */
    usart3_gpio_init();

    /* Port 0 : USART2 (PD5/PD6), dir PD7, DMA1_Stream6 Ch4. */
    port_init(&s_ports[0], USART2, &huart2,
              DMA1_Stream6, DMA_CHANNEL_4,
              RS485_TX_RX__GPIO_Port, RS485_TX_RX__Pin);

    /* Port 1 : USART3 (PC10/PC11), dir PD10 (P5_GPIOD10), DMA1_Stream3 Ch4. */
    port_init(&s_ports[1], USART3, &huart3,
              DMA1_Stream3, DMA_CHANNEL_4,
              P5_GPIOD10_GPIO_Port, P5_GPIOD10_Pin);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void DMX_SetSlot(uint8_t port, uint16_t idx, uint8_t value)
{
    if (port < DMX_NUM_PORTS && idx < DMX_SLOTS)
        s_ports[port].work[1 + idx] = value;
}

void DMX_SetSlots(uint8_t port, uint16_t start_slot, const uint8_t *data, uint16_t count)
{
    if (port >= DMX_NUM_PORTS || start_slot >= DMX_SLOTS || data == NULL)
        return;
    if (start_slot + count > DMX_SLOTS)
        count = DMX_SLOTS - start_slot;
    memcpy(&s_ports[port].work[1 + start_slot], data, count);
}

void DMX_Commit(uint8_t port)
{
    (void)port;  /* émission à cadence fixe : commit informatif */
}

bool DMX_Busy(uint8_t port)
{
    return (port < DMX_NUM_PORTS) ? s_ports[port].tx_busy : false;
}

/* Lance l'émission d'une trame sur un port : BREAK d'abord. */
static void port_start_frame(dmx_port_t *p)
{
    memcpy(p->frame, p->work, sizeof(p->frame));
    p->frame[0] = 0x00;   /* start code DMX (null) */

    p->tx_busy = true;
    DIR_TX(p);

    p->state = DMX_ST_BREAK;
    port_set_baud(p, DMX_BREAK_BAUD);
    HAL_UART_Transmit_DMA(p->huart, &s_break_byte, 1);
}

/* Avance la machine à états d'un port en fin de transmission. */
static void port_tx_cplt(dmx_port_t *p)
{
    if (p->state == DMX_ST_BREAK) {
        p->state = DMX_ST_DATA;
        port_set_baud(p, DMX_BAUD);
        HAL_UART_Transmit_DMA(p->huart, p->frame, sizeof(p->frame));
    } else if (p->state == DMX_ST_DATA) {
        p->state   = DMX_ST_IDLE;
        p->tx_busy = false;
        DIR_RX(p);   /* relâche le bus (RDM à l'étape 4) */
    }
}

/* Callback HAL de fin de transmission : dispatch vers le bon port. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < DMX_NUM_PORTS; i++) {
        if (s_ports[i].usart != NULL && huart->Instance == s_ports[i].usart) {
            port_tx_cplt(&s_ports[i]);
            return;
        }
    }
}

void DMX_DMA_TX_IRQHandler(uint8_t port)
{
    if (port < DMX_NUM_PORTS)
        HAL_DMA_IRQHandler(&s_ports[port].hdma_tx);
}

void DMX_Task(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < DMX_NUM_PORTS; i++) {
        dmx_port_t *p = &s_ports[i];
        if (p->state != DMX_ST_IDLE)
            continue;
        if ((now - p->last_refresh_ms) < DMX_REFRESH_MS)
            continue;
        p->last_refresh_ms = now;
        port_start_frame(p);
    }
}
