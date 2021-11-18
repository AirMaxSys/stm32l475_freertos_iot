/*
* Lowlevel driver for microSD
*   Init SPI bus
*   Init SPI CS PIN
*   Init DMA
*/

#include "sdmmc.h"
#include "stm32l4xx_hal.h"

struct __packed sdmmc_cmd_frame {
    uint8_t idx;
    union {
        uint8_t bytes[4];
        DWORD word;
    } arg;
    uint8_t crc; 
};
typedef struct sdmmc_cmd_framm sdmmc_cmd_frame_t;

/* sdmmc GPIOs of SPI bus definition */
#define SDMMC_CS_PORT   GPIOC
#define SDMMC_CS_PIN    GPIO_PIN_3
#define SDMMC_CLK_PORT  GPIOA
#define SDMMC_CLK_PIN   GPIO_PIN_5
#define SDMMC_DI_PORT   GPIOA
#define SDMMC_DI_PIN    GPIO_PIN_6
#define SDMMC_DO_PORT   GPIOA
#define SDMMC_DO_PIN    GPIO_PIN_7

/* sdmmc SPI bus and DMA declaration */
SPI_HandleTypeDef hspi_sdmmc;
DMA_HandleTypeDef hdma_sdmmc_tx;
DMA_HandleTypeDef hdma_sdmmc_rx;

static void sdmmc_cs_pin_init(void)
{
    // Enable CS PORT clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Pullup CS PIN(unselect)
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);

    // Initializition CS PIN
    GPIO_InitTypeDef cs = {0};
    cs.Pin      = SDMMC_CS_PIN;
    cs.Mode     = GPIO_MODE_OUTPUT_PP;
    cs.Pull     = GPIO_PULLUP;
    cs.Speed    = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init()
}

void sdmmc_cs_select(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_RESET);
}

void sdmmc_cs_unselect(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);
}

/**
 * 
 */
void sdmmc_spi_init(void)
{
    // Initializition SPI CS PIN
    sdmmc_cs_pin_init();

    // Enanle clock
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Initializition GPIOs of SPI bus
    GPIO_InitTypeDef hgpios = {0};
    hgpios.Pin          = SDMMC_CLK_PIN | SDMMC_DI_PIN | SDMMC_DO_PIN;
    hgpios.Mode         = GPIO_MODE_AF_PP;
    hgpios.Pull         = GPIO_PULLUP;
    hgpios.Speed        = GPIO_SPEED_FREQ_VERY_HIGH;
    hgpios.Alternate    = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &hgpios);

    // MicroSD connect to MCU with SPI1
    // MicorSD can work with SPI mode0 or mode 3
    hspi_sdmmc.Instance                 = SPI1;
    hspi_sdmmc.Init.CLKPolarity         = SPI_POLARITY_LOW;
    hspi_sdmmc.Init.CLKPhase            = SPI_PHASE_1EDGE;
    hspi_sdmmc.Init.Mode                = SPI_MODE_MASTER;
    hspi_sdmmc.Init.Direction           = SPI_DIRECTION_2LINES;
    hspi_sdmmc.Init.DataSize            = SPI_DATASIZE_8BIT;
    hspi_sdmmc.Init.BaudRatePrescaler   = SPI_BAUDRATEPRESCALER_2;
    hspi_sdmmc.Init.FirstBit            = SPI_FIRSTBIT_MSB;
    hspi_sdmmc.Init.NSS                 = SPI_NSS_SOFT;
    hspi_sdmmc.Init.NSSMode             = SPI_NSS_PULSE_ENABLE;
    hspi_sdmmc.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLE;
    hspi_sdmmc.Init.TIMode              = SPI_TIMODE_DISABLE;
    if (HAL_SPI_Init(&hspi_sdmmc) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }
}

void sdmmc_dma_init(void)
{

}

/**
 *  Host send one command to sdmmc
 * 
 *  @param cmd SD command definitions
 *  @param arg argument in SD command frame
 *  @return SDMM R0 response value
 */
uint8_t sdmmc_send_cmd(uint8_t cmd, DWORD arg)
{
    sdmmc_cmd_frame_t cmd_fm;
    uint8_t *uc_ptr = (uint8_t *)&cmd_fm;
    uint8_t dummy_txbuf = 0xFF;
    uint8_t rxbuf = 0xFF;

    // check if CMD is ACMD
    if (cmd & 0x80) {
        cmd_fm.idx = 0x40 | CMD55;
        cmd_fm.arg.word = 0;
        // Dummy CRC
        cmd_fm.crc = 0x1

        HAL_SPI_Transmit_IT(&hspi_sdmmc, uc_ptr, sizeof(sdmmc_cmd_frame_t));
        for (uint8_t i = 0; (rxbuf & 0x80) && (i < 10); ++i)
            HAL_SPI_TransmitReceive_IT(&hspi_sdmmc, &dummy_txbuf, &rxbuf, 1);
        if (rxbuf != 0) {
            // TODO:LOG error
            return rxbuf;
        }
        cmd &= 0x7F;
    }

    // CMD index: format 0b01+cmd
    cmd_fm.idx = 0x40 | cmd;
    // CMD argument: 32bits MSB (can only apply to liile-edain CPU)
    cmd_fm.arg.bytes[0] = (uint8_t)(arg >> 24);
    cmd_fm.arg.bytes[1] = (uint8_t)(arg >> 16);
    cmd_fm.arg.bytes[2] = (uint8_t)(arg >> 8);
    cmd_fm.arg.bytes[3] = (uint8_t)(arg >> 0);
    // CMD CRC:
    // CRC_CMD0(arg=0)=0x95
    // CRC_CMD8(arg=0x1AA)=0x87
    // CRC_dummy = 0x01
    cmd_fm.crc = (cmd == CMD0) ? 0x95 : (cmd == CMD8) ? 0x87 : 0x01;

    HAL_SPI_Transmit_IT(&hspi_sdmmc, uc_ptr, sizeof(sdmmc_cmd_frame_t));
    for (uint8_t i = 0; (rxbuf & 0x80) && (i < 10); ++i)
        HAL_SPI_TransmitReceive_IT(&hspi_sdmmc, &dummy_txbuf, &rxbuf, 1);
    return rxbuf;
}
