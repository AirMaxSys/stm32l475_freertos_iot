/**
  * Lowlevel driver for microSD
  *     Initilize SPI bus
  *     Initilize SPI CS PIN
  *     Initilize DMA
  *     Complete Fatfs disk IO API
  */

#include "mmc.h"
#include "diskio.h"
#include "stm32l475xx.h"
#include "stm32l4xx_hal.h"
#include "proj_config.h"

#if USING_FREERTOS == 1
#include "task.h"
#include "semphr.h"
#endif

#define MMC_DBG_ON
#ifdef MMC_DBG_ON
#include <stdio.h>
#endif

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

/* Fatfs chunk size*/
#define FATFS_CHUNK_SIZE    512

/* sdmmc structs decleration*/
struct sdmmc_cmd_frame {
    uint8_t idx;
    union {
        uint8_t bytes[4];
        uint32_t word;
    } arg;
    uint8_t crc; 
} __packed;
typedef struct sdmmc_cmd_frame sdmmc_cmd_frame_t;

/* sdmmc SPI bus and DMA declaration */
SPI_HandleTypeDef hspi_sdmmc;
DMA_HandleTypeDef hdma_sdmmc_tx;
DMA_HandleTypeDef hdma_sdmmc_rx;

/* sdmmc local variable*/
static volatile uint8_t fatfs_sdmmc_state = STA_NOINIT;
/* DMA sync local variable*/
#if USING_FREERTOS == 1
static SemaphoreHandle_t dmatx_sync_sem;
static SemaphoreHandle_t dmarx_sync_sem;
#else
static volatile uint8_t dmatx_sync_var;
static volatile uint8_t dmarx_sync_var;
#endif

/* SDMMC SPI DMA transmit and receive done iterrupt callback functions*/
void sdmmc_spi_dmatx_complete_cb(SPI_HandleTypeDef *hspi);
void sdmmc_spi_dmarx_complete_cb(SPI_HandleTypeDef *hspi);

/* sdmmc lowlevel functions definition*/

/**
 * @brief Initailize SPI bus CS PIN
 */
static void sdmmc_cs_pin_init(void)
{
    // Enable CS PORT clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Pullup CS PIN(unselect)
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);

    // Initialize CS PIN
    GPIO_InitTypeDef cs = {0};
    cs.Pin      = SDMMC_CS_PIN;
    cs.Mode     = GPIO_MODE_OUTPUT_PP;
    cs.Pull     = GPIO_PULLUP;
    cs.Speed    = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(SDMMC_CS_PORT, &cs);
}


/**
 * @brief Initialize and config SPI bus
 */
static void sdmmc_spi_init(void)
{
    // Initialize SPI CS PIN
    sdmmc_cs_pin_init();

    // Enanle clock
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Initialize GPIOs of SPI bus
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
    hspi_sdmmc.Init.NSSPMode            = SPI_NSS_PULSE_ENABLE;
    hspi_sdmmc.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLE;
    hspi_sdmmc.Init.TIMode              = SPI_TIMODE_DISABLE;
    if (HAL_SPI_Init(&hspi_sdmmc) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }

    // Enable SPI interrupt
    HAL_NVIC_SetPriority(SPI1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    // Register callback funtions
    HAL_SPI_RegisterCallback(&hspi_sdmmc, HAL_SPI_TX_COMPLETE_CB_ID, sdmmc_spi_dmatx_complete_cb);
    HAL_SPI_RegisterCallback(&hspi_sdmmc, HAL_SPI_RX_COMPLETE_CB_ID, sdmmc_spi_dmarx_complete_cb);
    
    // Create DMA TX completion counting semaphore
#if USING_FREERTOS == 1
    if (!(dmatx_sync_sem = xSemaphoreCreateCounting(1, 0))) {
        // TODO:LOG error
        __NOP();
    }
    if (!(dmarx_sync_sem = xSemaphoreCreateCounting(1, 0))) {
        // TODO:LOG error
        __NOP();
    }
#endif
}

/**
 * Initialize and config DMA with sdmmc in both TX
 *  and RX directions.
 */
static void sdmmc_dma_init(void)
{
    // Enable DMA clock
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Config DMA TX(MCU refrence manual table44)
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

    // Initialize
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
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 1);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief Pulldown SPI bus CS pin to ready to start communication 
 */
static inline void sdmmc_select(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Pullup SPI bus CS pin to stop communication
 */
static inline void sdmmc_unselect(void)
{
    HAL_GPIO_WritePin(SDMMC_CS_PORT, SDMMC_CS_PIN, GPIO_PIN_SET);
}

/**
 *  @brief Host send one command to sdmmc
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
        cmd_fm.crc = 0x1;

        HAL_SPI_Transmit(&hspi_sdmmc, u8_ptr, sizeof(sdmmc_cmd_frame_t), 0xFF);
        // NCR(command response time)
        for (uint8_t i = 0; (rxbuf & 0x80) && (i < 10); ++i)
            HAL_SPI_TransmitReceive(&hspi_sdmmc, &dummy_txbuf, &rxbuf, 1, 0xFF);
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
    
    HAL_SPI_Transmit(&hspi_sdmmc, u8_ptr, sizeof(sdmmc_cmd_frame_t), 0xFF);
    // CMD12 response R1b take time longer than NCR
    if (cmd == CMD12)
        HAL_SPI_Transmit(&hspi_sdmmc, &dummy_txbuf, 1, 0xFF);
    // NCR(command response time)
    for (uint8_t i = 0; (rxbuf & 0x80) && (i < 10); ++i) 
        HAL_SPI_TransmitReceive(&hspi_sdmmc, &dummy_txbuf, &rxbuf, 1, 0xFF);
    
    return rxbuf;
}

/**
 * @brief SPI transmit a mount of data using DMA
 * @param wbuf Data transfering buffer
 * @param toke SDMMC data packet token
 * @return 0:success 1:failer
 */
static int sdmmc_write(uint8_t *wbuf, uint8_t token)
{
    int res = 0;
    uint8_t dummy_rx[2] = {0};
    uint8_t dummy_tx[2] = {0xFF};
    uint8_t resp_val = 0;
    TimeOut_t timeout;
    TickType_t wait_tick = pdMS_TO_TICKS(200);

    // Send data if token not stop transmission
    if (token != 0xFD) {
        HAL_SPI_Transmit_DMA(&hspi_sdmmc, wbuf, FATFS_CHUNK_SIZE);
    #if USING_FREERTOS == 1
        xSemaphoreTake(dmatx_sync_sem, portMAX_DELAY);
    #else
        while (!dmatx_sync_var);
        dmatx_sync_var = 0
    #endif
        // Send dummy CRC
        HAL_SPI_TransmitReceive(&hspi_sdmmc, dummy_tx, dummy_rx, 2, 0xFF);
        // Get response data
        HAL_SPI_TransmitReceive(&hspi_sdmmc, &dummy_tx[0], &resp_val, 1, 0xFF);
        // Without response value
        if ((resp_val & 0x1F) != 0x05) {
            res = 1;
        }
    } else {
        HAL_SPI_Transmit(&hspi_sdmmc, &token, 1, 0xFF);
    }
    return res;
}

/**
 * @brief SPI receive a mount of data using DMA
 * @param rbuf Data receiving buffer
 * @param count The size of the data to be received
 * @return 0:success 1:failure
 */
static int sdmmc_read(uint8_t *rbuf, uint32_t count)
{
    uint8_t *ptr = rbuf;
    const uint16_t chunk = 65535;
    uint8_t token = 0;
    uint8_t dummy_rx[2] = {0};
    uint8_t dummy_tx[2] = {0xFF, 0xFF};
    // Fixme: add without using RTOS situation
    TimeOut_t timeout;
    TickType_t wait_tick = pdMS_TO_TICKS(200);

    // Get token while take some time(200ms)
    vTaskSetTimeOutState(&timeout);
    do {
        HAL_SPI_TransmitReceive(&hspi_sdmmc, &dummy_tx[0], &token, 1, 0xFF);
    } while ((token == 0xFF) && (xTaskCheckForTimeOut(&timeout, &wait_tick) == pdFALSE));
    if (token != 0xFE)
        return 1;

    // Receive datas 
    if (count > chunk) {
        while (count > chunk) {
            HAL_SPI_Receive_DMA(&hspi_sdmmc, ptr, chunk);
#if USING_FREERTOS == 1
            xSemaphoreTake(dmarx_sync_sem, portMAX_DELAY);
#else
            while (!dmarx_sync_var);
            dmarx_sync_var = 0
#endif
            count -= chunk;
            ptr += chunk;
        }
    }
    // HAL_SPI_Receive(&hspi_sdmmc, ptr, count, 0xFF);
    HAL_SPI_Receive_DMA(&hspi_sdmmc, ptr, count);
#if USING_FREERTOS == 1
    xSemaphoreTake(dmarx_sync_sem, portMAX_DELAY);
#else
    while (!dmarx_sync_var);
    dmarx_sync_var = 0
#endif
    // Discard 2bytes CRC
    HAL_SPI_TransmitReceive(&hspi_sdmmc, dummy_tx, dummy_rx, 2, 0xFF);

    return 0;
}

/*FatFs diskio layer MMC API defination*/

/**
 * @brief Lowlevel disk IO initialization, check if MMC card exist.
 * @return Disk IO driver state
 */
int MMC_disk_initialize(void)
{
    uint8_t ret = 0;
    uint8_t dummy_tx = 0xFF;
#if USING_FREERTOS == 1
    TimeOut_t init_timeout;
    TickType_t tickstowait = pdMS_TO_TICKS(1000);
#endif

    // Initilize MCU related peripheral
    sdmmc_spi_init();
    sdmmc_dma_init();

    // SDC/MMC power on sequence
    //  dummy clock (CS=DI=high)
    for (uint8_t i = 0; i < 10; ++i)
        HAL_SPI_Transmit(&hspi_sdmmc, &dummy_tx, 1, 0xFF);

    sdmmc_select();
    // Software reset
    if (sdmmc_send_cmd(CMD0, 0) == 0x01) {
        // Fixme: add without using RTOS situation
        vTaskSetTimeOutState(&init_timeout);
        // Initiate initialization process
        while (((ret = sdmmc_send_cmd(CMD1, 0)) == 0x01) &&  \
            (xTaskCheckForTimeOut(&init_timeout, &tickstowait) == pdFALSE));
        // Check detect SD card type is MMC
        if ((ret == 0) && (xTaskCheckForTimeOut(&init_timeout, &tickstowait) == pdFALSE)) {
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
 * @brief Get disk state
 * @return Disk IO state
 */
int MMC_disk_status(void)
{
    return fatfs_sdmmc_state;
}

/**
 * @brief Read sector date to buffer
 * @param buff Read buffer to store read data
 * @param sector Sector addres
 * @param count Sector numbers
 * @return 0:success 1:failure
 */
int MMC_disk_read(uint8_t *buff, uint32_t sector, uint32_t count)
{
    int res = 0;
    uint8_t cmd;
    uint8_t *ptr = buff;
    uint32_t pcount = count;

    // Fatfs block size 512bytes
    sector *= FATFS_CHUNK_SIZE;
    // Single or multiple block read
    cmd = (count == 1) ? CMD17 : CMD18;

    sdmmc_select();
    // Send command to SDMMC
    if (sdmmc_send_cmd(cmd, sector) != 0)
        res = 1;
    // Receive data
    do {
        if (sdmmc_read(ptr, FATFS_CHUNK_SIZE) != 0) {
            res = 1;
            break;
        }
        ptr += FATFS_CHUNK_SIZE; 
    } while (--pcount);
    // Send stop command when multiple block read
    if (count > 1)
        sdmmc_send_cmd(CMD12, 0);
    sdmmc_unselect();

    return res;
}

/**
 * @brief Write buffer data to SD card block
 * @param buff write buffer to store datas
 * @param sector Sector addres
 * @param count Sector numbers
 * @return 0:success 1:failure
 */
int MMC_disk_write(const uint8_t *buff, uint32_t sector, uint32_t count)
{
    int res = 0;
    uint8_t cmd, token;
    uint8_t *ptr = (uint8_t *)buff;
    
    // Fatfs block size 512bytes
    sector *= FATFS_CHUNK_SIZE;
    // Single or multiple block read
    cmd = (count == 1) ? CMD24 : CMD25;
    token = (count == 1) ? 0xFE : 0xFC;

    sdmmc_select();
    // Send write single or multiple block command
    if (sdmmc_send_cmd(cmd, sector) != 0)
        res = 1;
    // Transfer data
    do {
        if (sdmmc_write(ptr, token) != 0) {
            res = 1;
            break;
        }
    } while (--count);
    // Stop transfering token
    token = 0xFD;
    if (sdmmc_write(0, token) != 0)
        res = 1;
    sdmmc_unselect();

    return res;
}

/**
 * @brief SD card some basic control functions
 * @param cmd Generic commands used by Fatfs
 * @param buff Buffer to send or receive control data
 * @return  RES_OK succes
 *          RES_ERROR failure
 *          RES_PARERR unsupported command
 */
int MMC_disk_ioctl(uint8_t cmd, void *buff)
{
    int res = RES_ERROR;
    uint8_t csd[16] = {0};
    uint8_t n = 0;
    uint32_t csize = 0;

    switch (cmd)
    {
    case CTRL_SYNC:
        sdmmc_select();
        res = RES_OK;
        break;
    case GET_SECTOR_COUNT:
        if ((sdmmc_send_cmd(CMD9, 0) == 0) && (sdmmc_read(csd, 16) == 0)) {
            if ((csd[0] >> 6) != 1) {
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((uint32_t)csd[7] << 2) + ((uint32_t)(csd[6] & 3) << 10) + 1;
                *(uint32_t *)buff = csize << (n - 9);
				*(uint32_t *)buff = csize << (n - 9); 
                *(uint32_t *)buff = csize << (n - 9);
            }
        }
        res = RES_OK;
        break;
    case GET_BLOCK_SIZE:
        if ((sdmmc_send_cmd(CMD9, 0) == 0) && (sdmmc_read(csd, 16) == 0)) {
            *(uint32_t *)buff = ((uint32_t)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
        }
        res = RES_OK;
        break;
    
    default:
        res = RES_PARERR;
        break;
    }
    sdmmc_unselect();
    return res;
}

/**
 * @brief
*/
void sdmmc_spi_dmatx_complete_cb(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi_sdmmc) {
#if USING_FREERTOS == 1
        BaseType_t task_need_woken;
        xSemaphoreGiveFromISR(dmatx_sync_sem, &task_need_woken);
#else
        dmatx_sync_var = 1;
#endif
    }
}

/**
 * @brief
*/
void sdmmc_spi_dmarx_complete_cb(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi_sdmmc) {
#if USING_FREERTOS == 1
        BaseType_t task_need_woken;
        xSemaphoreGiveFromISR(dmarx_sync_sem, &task_need_woken);
#else
        dmarx_sync_var = 1;
#endif
    }
}

/**
 * @brief SPI1 interrupt handle function
*/
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi_sdmmc);
}

/**
 * @brief DMA1 channel2 interrupt handle function
*/
void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sdmmc_rx);
}

/**
 * @brief DMA1 channel3 interrupt handle function
*/
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sdmmc_tx);
}
