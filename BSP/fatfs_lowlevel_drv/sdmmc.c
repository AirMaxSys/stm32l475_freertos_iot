/**
  * Lowlevel driver for microSD
  *   Init SPI bus
  *   Init SPI CS PIN
  *   Init DMA
  */

#include "sdmmc.h"
#include "diskio.h"
#include "stm32l475xx.h"
#include "stm32l4xx_hal.h"

#include "task.h"

/* sdmmc GPIOs of SPI bus definition */
#define SDMMC_CS_PORT   GPIOC
#define SDMMC_CS_PIN    GPIO_PIN_3
#define SDMMC_CLK_PORT  GPIOA
#define SDMMC_CLK_PIN   GPIO_PIN_5
#define SDMMC_DI_PORT   GPIOA
#define SDMMC_DI_PIN    GPIO_PIN_6
#define SDMMC_DO_PORT   GPIOA
#define SDMMC_DO_PIN    GPIO_PIN_7

/* SDC/MMC commands definitaion*/
#define CMD0        (0)         /* GO_IDLE_STATE */
#define CMD1        (1)         /* SEND_OP_COND (MMC) */
#define ACMD41      (0x80 + 41) /* SEND_OP_COND (SDC) */
#define CMD8        (8)         /* SEND_IF_COND */
#define CMD9        (9)         /* SEND_CSD */
#define CMD10       (10)        /* SEND_CID */
#define CMD12       (12)        /* STOP_TRANSMISSION */
#define ACMD13      (0x80 + 13) /* SD_STATUS (SDC) */
#define CMD16       (16)        /* SET_BLOCKLEN */
#define CMD17       (17)        /* READ_SINGLE_BLOCK */
#define CMD18       (18)        /* READ_MULTIPLE_BLOCK */
#define CMD23       (23)        /* SET_BLOCK_COUNT (MMC) */
#define ACMD23      (0x80 + 23) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24       (24)        /* WRITE_BLOCK */
#define CMD25       (25)        /* WRITE_MULTIPLE_BLOCK */
#define CMD32       (32)        /* ERASE_ER_BLK_START */
#define CMD33       (33)        /* ERASE_ER_BLK_END */
#define CMD38       (38)        /* ERASE */
#define CMD55       (55)        /* APP_CMD */
#define CMD58       (58)        /* READ_OCR */

/* sdmmc structs decleration*/
struct __packed sdmmc_cmd_frame {
    uint8_t idx;
    union {
        uint8_t bytes[4];
        uint32_t word;
    } arg;
    uint8_t crc; 
};
typedef struct sdmmc_cmd_framm sdmmc_cmd_frame_t;

/* sdmmc SPI bus and DMA declaration */
SPI_HandleTypeDef hspi_sdmmc;
DMA_HandleTypeDef hdma_sdmmc_tx;
DMA_HandleTypeDef hdma_sdmmc_rx;

/* sdmmc local veraibles*/
volatile static fatfs_sdmmc_state = STA_NOINIT;

/* sdmmc lowlevel functions definition*/

/**
 * Initailize SPI bus CS PIN
 */
static void sdmmc_cs_pin_init(void)
{
    // Enable CS PORT clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Pullup CS PIN(unselect)
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);

    // Initialization CS PIN
    GPIO_InitTypeDef cs = {0};
    cs.Pin      = SDMMC_CS_PIN;
    cs.Mode     = GPIO_MODE_OUTPUT_PP;
    cs.Pull     = GPIO_PULLUP;
    cs.Speed    = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init()
}

/**
 * Pulldown SPI bus CS pin to ready to start communication 
 */
static inline void sdmmc_select(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_RESET);
}

/**
 * Pullup SPI bus CS pin to stop communication
 */
static inline void sdmmc_unselect(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);
}

/**
 * Initialize and config SPI bus
 */
static void sdmmc_spi_init(void)
{
    // Initialization SPI CS PIN
    sdmmc_cs_pin_init();

    // Enanle clock
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Initialization GPIOs of SPI bus
    GPIO_InitTypeDef hgpios = {0};
    hgpios.Pin          = SDMMC_CLK_PIN | SDMMC_DI_PIN | SDMMC_DO_PIN;
    hgpios.Mode         = GPIO_MODE_AF_PP;
    hgpios.Pull         = GPIO_PULLUP;
    hgpios.Speed        = GPIO_SPEED_FREQ_VERY_HIGH;
    hgpios.Alternate    = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &hgpios);

    // MicroSD connect to MCU with SPI1
    // SDC/MMC can work with SPI mode0 or mode 3
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

    // Enable SPI interrupt
    HAL_NVIC_SetPriority(SPI1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/**
 * Initialize and config DMA with sdmmc in both TX
 *  and RX directions.
 */
static void sdmmc_dma_init(void)
{
    // Enable DMA clock
    __HAL_RCC_DMA1_CLK_ENABLE();

    // config DMA TX(MCU refrence manual table44)
    hdma_sdmmc_tx.Instance                  = DMA1_Channel3;
    hdma_sdmmc_tx.Init.Request              = DMA_REQUEST_1;
    hdma_sdmmc_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;
    // SPIx_DR register is 16-bites width
    hdma_sdmmc_tx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    hdma_sdmmc_tx.Init.PeriphInc            = DMA_PINC_DISABLE;
    hdma_sdmmc_tx.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    hdma_sdmmc_tx.Init.MemInc               = DMA_MINC_ENABLE;
    hdma_sdmmc_tx.Init.Mode                 = DMA_NORMAL;
    hdma_sdmmc_tx.Init.Priority             = DMA_PRIORITY_MEDIUM;

    // config DMA RX
    hdma_sdmmc_rx.Instance                  = DMA1_Channel2;
    hdma_sdmmc_rx.Init.Request              = DMA_REQUEST_1;
    hdma_sdmmc_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
    hdma_sdmmc_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    hdma_sdmmc_rx.Init.PeriphInc            = DMA_PINC_DISABLE;
    hdma_sdmmc_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    hdma_sdmmc_rx.Init.MemInc               = DMA_MINC_ENABLE;
    hdma_sdmmc_rx.Init.Mode                 = DMA_NORMAL;
    hdma_sdmmc_rx.Init.Priority             = DMA_PRIORITY_MEDIUM;

    // Initialization
    if (HAL_DMA_Init(&hdma_sdmmc_tx) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }
    if (HAL_DMA_Init(&hdma_sdmmc_rx) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }

    // Link SPI TX with DMA
    __HAL_LINKDMA(&hspi_sdmmc, hdmatx, hdma_sdmmc_tx);
    __HAL_LINKDMA(&hspi_sdmmc, hdmarx, hdma_sdmmc_rx);

    // Enable DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 1);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 *  Host send one command to sdmmc
 * 
 *  @param cmd SD command definitions
 *  @param arg argument in SD command frame
 *  @return SDMM R1 response value
 */
static uint8_t sdmmc_send_cmd(uint8_t cmd, uint32_t arg)
{
    sdmmc_cmd_frame_t cmd_fm;
    uint8_t *u8_ptr = (uint8_t *)&cmd_fm;
    uint8_t dummy_txbuf = 0xFF;
    uint8_t rxbuf = 0xFF;

    // check if CMD is ACMD
    if (cmd & 0x80) {
        cmd_fm.idx = 0x40 | CMD55;
        cmd_fm.arg.word = 0;
        // Dummy CRC
        cmd_fm.crc = 0x1

        HAL_SPI_Transmit_IT(&hspi_sdmmc, u8_ptr, sizeof(sdmmc_cmd_frame_t));
        // NCR(command response time)
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
    // CMD argument: 32bits MSB (can only apply to little-edian CPU)
    cmd_fm.arg.bytes[0] = (uint8_t)(arg >> 24);
    cmd_fm.arg.bytes[1] = (uint8_t)(arg >> 16);
    cmd_fm.arg.bytes[2] = (uint8_t)(arg >> 8);
    cmd_fm.arg.bytes[3] = (uint8_t)(arg >> 0);
    // CMD CRC:
    //  CRC_CMD0(arg=0)=0x95
    //  CRC_CMD8(arg=0x1AA)=0x87
    //  CRC_dummy = 0x01
    cmd_fm.crc = (cmd == CMD0) ? 0x95 : (cmd == CMD8) ? 0x87 : 0x01;

    HAL_SPI_Transmit_IT(&hspi_sdmmc, u8_ptr, sizeof(sdmmc_cmd_frame_t));
    // CMD12 response R1b take time longer than NCR
    if (cmd == CMD12) {
        HAL_SPI_Transmit_IT(&hspi_sdmmc, &dummy_txbuf, 1);
    }
    // NCR(command response time)
    for (uint8_t i = 0; (rxbuf & 0x80) && (i < 10); ++i)
        HAL_SPI_TransmitReceive_IT(&hspi_sdmmc, &dummy_txbuf, &rxbuf, 1);
    return rxbuf;
}

/*FatFs diskio layer MMC API defination*/

/**
 * @brief Lowlevel disk IO initialization, check if MMC card exist.
 * @return Disk IO driver state
 */
int MMC_disk_initialize(void)
{
    TimeOut_t init_timeout;
    TickType_t tickstowait = pdMS_TO_TICKS(1000);
    uint8_t res = 0;
    uint8_t dummy_tbuf[10] = {0xFF};

    // Initilize MCU related peripheral
    sdmmc_spi_init();
    sdmmc_dma_init();

    // SDC/MMC power on sequence
    //  dummy clock (CS=DI=high)
    HAL_SPI_Transmit_IT(&hspi_sdmmc, &dummy_tbuf; 10);

    sdmmc_select();
    // Software reset
    if (sdmmc_send_cmd(CMD0, 0) == 0x01) {
        vTaskSetTimeOutState(&init_timeout);
        // Initiate initialization process
        while (((res = sdmmc_send_cmd(CMD1, 0)) == 0x01) &&  \
            (xTaskCheckForTimeOut(&init_timeout, &tickstowait) == pdFALSE))
            ;
        // Check detect SD card type is MMC
        if ((res == 0) && (xTaskCheckForTimeOut(&init_timeout, &Tickstowait) != pdTRUE)) {
            // Set block size to 512bytes to work with FATFS
            sdmmc_send_cmd(CMD16, 0x200);
            fatfs_sdmmc_state &= (~STA_NOINIT);
        } else {
            // TODO: LOG error
            __NOP();
        }
    } else {
        //TODO: LOG error
        __NOP();
    }
    sdmmc_unselect();

    return fatfs_sdmmc_state;
}

/**
 * @brief
 * @return
 */
int MMC_disk_status(void)
{

}

/**
 * @brief
 * @param buff
 * @param sector
 * @param count
 * @return
 */
int MMC_disk_read(uint8_t *buff, uint32_t sector, uint32_t count)
{

}

/**
 * @brief
 * @param buff
 * @param sector
 * @param count
 * @return
 */
int MMC_disk_write(const uint8_t *buff, uint32_t sector, uint32_t count)
{

}

/**
 * @brief
 * @param cmd
 * @param buff
 * @return
 */
int MMC_disk_ioctl(uint8_t cmd, void *buff)
{

}
