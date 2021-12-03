/**
  * Lowlevel driver for microSD
  *     Initilize SPI bus
  *     Initilize SPI CS PIN
  *     Initilize DMA
  *     Complete Fatfs disk IO API
  */
#include <stdio.h>
#include <string.h>

#include "mmc.h"
#include "diskio.h"
#include "stm32l475xx.h"
#include "stm32l4xx_hal.h"
#include "proj_config.h"

#if USING_FREERTOS == 1
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#endif

/* SD card GPIO definition*/
#define SD_CS_PORT   GPIOC
#define SD_CS_PIN    GPIO_PIN_3
#define SD_CLK_PORT  GPIOA
#define SD_CLK_PIN   GPIO_PIN_5
#define SD_DI_PORT   GPIOA
#define SD_DI_PIN    GPIO_PIN_6
#define SD_DO_PORT   GPIOA
#define SD_DO_PIN    GPIO_PIN_7

/* SPI and DMA interrupt priority definitaion*/
#define SD_SPI1_IRQ_PRIORITY    (9) 
#define SD_DMA1_IRQ_PRIORITY    (9) 

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

/* SD card type definations*/
#define CT_UNKNOWN  0x00    /* Unknown type*/
#define CT_MMC      0x01    /* MMC */
#define CT_SDv1     0x02    /* SDC v1*/
#define CT_SDv2     0x04    /* SDC v2*/
#define CT_SDC      (CT_SDv1 | CT_SDv2) /* SDC*/
#define CT_BLOCK    0x08    /* SDC block addressing*/

/* Fatfs chunk size*/
#define FATFS_CHUNK_SIZE    512

/* SDC/MMC command data frame structs decleration*/
struct sd_cmd_frame {
    uint8_t idx;
    union {
        uint8_t bytes[4];
        uint32_t word;
    } arg;
    uint8_t crc; 
} __packed;
typedef struct sd_cmd_frame sd_cmd_frame_t;

/* SDC/MMC SPI bus and DMA declaration */
SPI_HandleTypeDef hspi_sd;
DMA_HandleTypeDef hdma_sd_tx;
DMA_HandleTypeDef hdma_sd_txrx;

/* SDC/MMC local variable*/
static volatile uint8_t sd_card_state = STA_NOINIT;
static volatile uint8_t sd_card_type = CT_UNKNOWN;
/* DMA sync local variable*/
#if USING_FREERTOS == 1
static SemaphoreHandle_t dmatx_sync_sem;
static SemaphoreHandle_t dmatxrx_sync_sem;
static TimeOut_t timeout;
static TickType_t tickstowait;
#else
static volatile uint8_t dmatx_sync_var;
static volatile uint8_t dmatxrx_sync_var;
static uint32_t start_time;
static uint16_t timetowait;
#endif

/* SDC/MMC SPI DMA transmit and receive done iterrupt callback functions*/
void sd_spi_dmatx_complete_cb(SPI_HandleTypeDef *hspi);
void sd_spi_dmatxrx_complete_cb(SPI_HandleTypeDef *hspi);

/* Check timeout functions with FreeRTOS or bare metal system*/

/**
 * @brief Initialize timeout state
*/
static inline void init_timeout_state(void)
{
#if USING_FREERTOS == 1
    vTaskSetTimeOutState(&timeout);
#else
    start_time = HAL_GetTick();
#endif
}

/**
 * @brief Set timeout value
*/
static inline void set_timetowait_val(uint32_t ms)
{
#if USING_FREERTOS == 1
    tickstowait = pdMS_TO_TICKS(ms);
#else
    timetowait = ms;
#endif
}

/**
 * @brief Check timeout state
 * @return 1: timeout occurred 0: without timeout
*/
static inline uint8_t check_timeout(void)
{
#if USING_FREERTOS == 1
    return (xTaskCheckForTimeOut(&timeout, &tickstowait) == pdTRUE) ? 1 : 0;
#else
    return (HAL_GetTick() - start_time >= timetowait) ? 1 : 0;
#endif
}

/* SDC/MMC lowlevel functions definition*/

/**
 * @brief Initailize SPI bus CS PIN
 */
static void sd_cs_pin_init(void)
{
    // Enable CS PORT clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Pullup CS PIN(unselect)
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);

    // Initialize CS PIN
    GPIO_InitTypeDef cs = {0};
    cs.Pin      = SD_CS_PIN;
    cs.Mode     = GPIO_MODE_OUTPUT_PP;
    cs.Pull     = GPIO_PULLUP;
    cs.Speed    = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(SD_CS_PORT, &cs);
}

/**
 * @brief Initialize and config SPI bus
 */
static void sd_spi_init(void)
{
    // Initialize SPI CS PIN
    sd_cs_pin_init();

    // Enanle clock
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Initialize GPIOs of SPI bus
    GPIO_InitTypeDef hgpios = {0};
    hgpios.Pin          = SD_CLK_PIN | SD_DI_PIN | SD_DO_PIN;
    hgpios.Mode         = GPIO_MODE_AF_PP;
    hgpios.Pull         = GPIO_PULLUP;
    hgpios.Speed        = GPIO_SPEED_FREQ_VERY_HIGH;
    hgpios.Alternate    = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &hgpios);

    // MicroSD connect to MCU with SPI1
    // SDC/MMC can work with SPI mode0 or mode 3
    hspi_sd.Instance                 = SPI1;
    hspi_sd.Init.CLKPolarity         = SPI_POLARITY_LOW;
    hspi_sd.Init.CLKPhase            = SPI_PHASE_1EDGE;
    hspi_sd.Init.Mode                = SPI_MODE_MASTER;
    hspi_sd.Init.Direction           = SPI_DIRECTION_2LINES;
    hspi_sd.Init.DataSize            = SPI_DATASIZE_8BIT;
    hspi_sd.Init.BaudRatePrescaler   = SPI_BAUDRATEPRESCALER_2;
    hspi_sd.Init.FirstBit            = SPI_FIRSTBIT_MSB;
    hspi_sd.Init.NSS                 = SPI_NSS_SOFT;
    hspi_sd.Init.NSSPMode            = SPI_NSS_PULSE_ENABLE;
    hspi_sd.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLE;
    hspi_sd.Init.TIMode              = SPI_TIMODE_DISABLE;
    if (HAL_SPI_Init(&hspi_sd) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }

    // Enable SPI interrupt
    HAL_NVIC_SetPriority(SPI1_IRQn, SD_SPI1_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    // Register callback funtions
    HAL_SPI_RegisterCallback(&hspi_sd, HAL_SPI_TX_COMPLETE_CB_ID, sd_spi_dmatx_complete_cb);
    HAL_SPI_RegisterCallback(&hspi_sd, HAL_SPI_TX_RX_COMPLETE_CB_ID, sd_spi_dmatxrx_complete_cb);
    
    // Create DMA TX completion counting semaphore
#if USING_FREERTOS == 1
    if (!(dmatx_sync_sem = xSemaphoreCreateCounting(1, 0))) {
        // TODO:LOG error
        __NOP();
    }
    if (!(dmatxrx_sync_sem = xSemaphoreCreateCounting(1, 0))) {
        // TODO:LOG error
        __NOP();
    }
#endif
}

/**
 * Initialize and config DMA in both TX/RX directions.
 */
static void sd_dma_init(void)
{
    // Enable DMA clock
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Config DMA TX(MCU refrence manual table44)
    hdma_sd_tx.Instance                  = DMA1_Channel3;
    hdma_sd_tx.Init.Request              = DMA_REQUEST_1;
    hdma_sd_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;
    // SPIx_DR register is 16-bites width
    hdma_sd_tx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    hdma_sd_tx.Init.PeriphInc            = DMA_PINC_DISABLE;
    hdma_sd_tx.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    hdma_sd_tx.Init.MemInc               = DMA_MINC_ENABLE;
    hdma_sd_tx.Init.Mode                 = DMA_NORMAL;
    hdma_sd_tx.Init.Priority             = DMA_PRIORITY_MEDIUM;

    // config DMA RX
    hdma_sd_txrx.Instance                  = DMA1_Channel2;
    hdma_sd_txrx.Init.Request              = DMA_REQUEST_1;
    hdma_sd_txrx.Init.Direction            = DMA_PERIPH_TO_MEMORY;
    hdma_sd_txrx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    hdma_sd_txrx.Init.PeriphInc            = DMA_PINC_DISABLE;
    hdma_sd_txrx.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    hdma_sd_txrx.Init.MemInc               = DMA_MINC_ENABLE;
    hdma_sd_txrx.Init.Mode                 = DMA_NORMAL;
    hdma_sd_txrx.Init.Priority             = DMA_PRIORITY_MEDIUM;

    // Initialize
    if (HAL_DMA_Init(&hdma_sd_tx) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }
    if (HAL_DMA_Init(&hdma_sd_txrx) != HAL_OK) {
        // TODO: LOG error
        __NOP();
    }

    // Link SPI TX with DMA
    __HAL_LINKDMA(&hspi_sd, hdmatx, hdma_sd_tx);
    __HAL_LINKDMA(&hspi_sd, hdmarx, hdma_sd_txrx);

    // Enable DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, SD_DMA1_IRQ_PRIORITY, 1);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, SD_DMA1_IRQ_PRIORITY, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief Pulldown SPI bus CS pin to ready to start communication 
 */
static inline void sd_select(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Pullup SPI bus CS pin to stop communication
 */
static inline void sd_unselect(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Wait SD card internal processing untill MISO to high(busy flag)
 * @return 0:success(not timeout) 1:failure(timeout)
*/
static uint8_t sd_wait_card_busyflag(void)
{
    uint8_t dummytx = 0xFF;
    uint8_t rx = 0;

    // Set wait SD card processing done timeout 500ms
    init_timeout_state();
    set_timetowait_val(500);

    // Wait MISO goes high level
    do {
        HAL_SPI_TransmitReceive(&hspi_sd, &dummytx, &rx, 1, 0xFF);
        if (rx == 0xFF)
            break;
    } while (check_timeout() == 0);

    return (rx == 0xFF) ? 0 : 1;
}

/**
 *  @brief Host send one command to SD card
 *  @param cmd SD command definitions
 *  @param arg argument in SD command frame
 *  @return SDC/MMC R1 response value
 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
    sd_cmd_frame_t cmd_fm;
    uint8_t *u8_ptr = (uint8_t *)&cmd_fm;
    uint8_t dummy = 0xFF;
    uint8_t resp = 0xFF;

    // check CMD is ACMD
    if (cmd & 0x80) {
        cmd &= 0x7F;
        resp = sd_send_cmd(CMD55, 0);
        if (resp > 1)
            return resp;
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
    HAL_SPI_Transmit(&hspi_sd, u8_ptr, sizeof(sd_cmd_frame_t), 0xFF);

    // CMD12 response R1b take time longer than NCR
    if (cmd == CMD12)
        HAL_SPI_Transmit(&hspi_sd, &dummy, 1, 0xFF);
    
    resp = 0xFF;
    // Get command response data
    for (uint8_t i = 0; (resp & 0x80) && (i < 10); ++i) 
        HAL_SPI_TransmitReceive(&hspi_sd, &dummy, &resp, 1, 0xFF);
    
    return resp;
}

/**
 * @brief SPI transmit a mount of data using DMA
 * @param wbuf Data transfering buffer
 * @param toke SDC/MMC data packet token
 * @return 0:success 1:failure
 */
static int sd_write(uint8_t *wbuf, uint8_t token)
{
    int res = 0;
    uint8_t dummy_rx[2] = {0};
    uint8_t dummy_tx[2] = {0xFF, 0xFF};
    uint8_t resp_val = 0;

    // Send token 
    HAL_SPI_Transmit(&hspi_sd, &token, 1, 0xFF);
    // Send data if token not stop transmission
    if (token != 0xFD) {
        HAL_SPI_Transmit_DMA(&hspi_sd, wbuf, FATFS_CHUNK_SIZE);
#if USING_FREERTOS == 1
        xSemaphoreTake(dmatx_sync_sem, portMAX_DELAY);
#else
        while (!dmatx_sync_var);
        dmatx_sync_var = 0;
#endif
        // Send dummy CRC
        HAL_SPI_TransmitReceive(&hspi_sd, dummy_tx, dummy_rx, 2, 0xFF);
        // Get response data
        HAL_SPI_TransmitReceive(&hspi_sd, &dummy_tx[0], &resp_val, 1, 0xFF);
        // Without response value
        if ((resp_val & 0x1F) != 0x05) {
            res = 1;
        }
    } else {
        HAL_SPI_Transmit(&hspi_sd, &token, 1, 0xFF);
    }
    return res;
}

/**
 * @brief SPI receive a mount of data using DMA
 * @param rbuf Data receiving buffer
 * @param count The size of the data to be received
 *      In Fatfs count less than 512bytes
 * @return 0:success 1:failure
 */
static int sd_read(uint8_t *rbuf, uint32_t count)
{
    uint8_t *ptr = rbuf;
    const uint16_t chunk = 65535;
    uint8_t token = 0;
    uint8_t dummy_rx[2] = {0};
    uint8_t dummy_tx[FATFS_CHUNK_SIZE];

    // Set dummy TX data value to 0xFF make MOSI pin keep in high state
    memset(dummy_tx, 0xFF, FATFS_CHUNK_SIZE);

    // Setup timeout 200 ms
    set_timetowait_val(200);
    init_timeout_state();

    // Get token value
    do {
        HAL_SPI_TransmitReceive(&hspi_sd, &dummy_tx[0], &token, 1, 0xFF);
    } while ((token == 0xFF) && (check_timeout() == 0));
    if (token != 0xFE)
        return 1;

    // Get data and wait DMA sync
    HAL_SPI_TransmitReceive_DMA(&hspi_sd, dummy_tx, ptr, count);
#if USING_FREERTOS == 1
    xSemaphoreTake(dmatxrx_sync_sem, portMAX_DELAY);
#else
    while (!dmatxrx_sync_var);
    dmatxrx_sync_var = 0;
#endif
    // Discard 2bytes CRC
    HAL_SPI_TransmitReceive(&hspi_sd, dummy_tx, dummy_rx, 2, 0xFF);

    return 0;
}

/*FatFs diskio layer MMC API defination*/

/**
 * @brief Lowlevel disk IO initialization check card type
 * @return Disk IO driver state
 */
int MMC_disk_initialize(void)
{
    uint8_t ret = 0;
    uint8_t card_type = 0;
    uint8_t dummy_tx = 0xFF;
    uint8_t ocr[4] = {0};

    // Initilize MCU related peripheral
    sd_spi_init();
    sd_dma_init();

    // Send dummy clock (CS=DI=high)
    for (uint8_t i = 0; i < 10; ++i)
        HAL_SPI_Transmit(&hspi_sd, &dummy_tx, 1, 0xFF);

    // SDC/MMC initialize sequence
    sd_select();
    // Software reset
    if (sd_send_cmd(CMD0, 0) == 0x01) {
        // Check voltage range
        if (sd_send_cmd(CMD8, 0x01AA) == 0x01) {
            // Get crad responsed 4bytes R7
            HAL_SPI_Receive(&hspi_sd, ocr, 4, 0xFF);

            // Setup timeout value detect card type takes about 1000 ms
            set_timetowait_val(1000);
            // Start tick count
            init_timeout_state();

            // Is card support 2.7-3.6 voltage
            if (0x1AA == (((uint16_t)ocr[2] << 8) | ocr[3])) {
                // Initiate initialization process for only SDC with timeout check
                while ((ret = sd_send_cmd(ACMD41, 0x40000000U)) && (check_timeout() == 0));
                // Check ACMD41 response value and timeout state
                if ((ret == 0) && (check_timeout() == 0)) {
                    // Get OCR value checkout card type is SDv2
                    if (sd_send_cmd(CMD58, 0) == 0) {
                        HAL_SPI_Transmit(&hspi_sd, ocr, 4, 0xFF);
                        if (ocr[0] & 0x40) {
                            card_type = (CT_SDv2 | CT_BLOCK);
                        } else {
                            card_type = (sd_send_cmd(CMD16, 0x200) == 0) ? CT_SDv2 : CT_UNKNOWN;
                        }
                    }
                }
            }
        } else { // SDv1 or MMC card
            while ((ret = sd_send_cmd(ACMD41, 0)) && (check_timeout() == 0));
            // Check ACMD41 response value and timeout state
            if ((ret == 0) && (check_timeout() == 0)) {
                // Set block size to 512bytes to work with FATFS
                card_type = (sd_send_cmd(CMD16, 0x200) == 0) ? CT_SDv1 : CT_UNKNOWN;
            } else {    // MMC card or unknown card
                while ((ret = sd_send_cmd(CMD1, 0)) && (check_timeout() == 0));
                // Check CMD1 response value and timeout state
                if ((ret == 0) && (check_timeout() == 0)) {
                    // Set block size to 512bytes to work with FATFS
                    card_type = (sd_send_cmd(CMD16, 0x200) == 0) ? CT_MMC : CT_UNKNOWN;
                }
            }
        }
    } else {
        //TODO: LOG error
        __NOP();
    }
    sd_unselect();
    
    // Detect card type reset disk mask bit
    if (card_type) {
        sd_card_type = card_type;
        sd_card_state &= ~STA_NOINIT;
    }

    return sd_card_state;
}

/**
 * @brief Get disk state
 * @return Disk IO state
 */
int MMC_disk_status(void)
{
    return sd_card_state;
}

/**
 * @brief Read sector date to buffer
 * @param buff Read buffer to store read data
 * @param sector Sector addres
 * @param count Sector numbers
 * @return RES_OK(0):success RES_ERROR(1):failure
 */
int MMC_disk_read(uint8_t *buff, uint32_t sector, uint32_t count)
{
    int res = RES_OK;
    uint8_t cmd;
    uint8_t *ptr = buff;
    uint32_t pcount = count;

    // Fatfs block size 512bytes
    if (!(sd_card_type & CT_BLOCK))
        sector *= FATFS_CHUNK_SIZE;
    // Single or multiple block read
    cmd = (count == 1) ? CMD17 : CMD18;

    sd_select();
    // Send command to SD card
    if (sd_send_cmd(cmd, sector) != 0)
        res = RES_ERROR;
    // Receive datas
    do {
        if (sd_read(ptr, FATFS_CHUNK_SIZE) != 0) {
            res = RES_ERROR;
            break;
        }
        ptr += FATFS_CHUNK_SIZE; 
    } while (--pcount);
    // Send stop command when multiple block read
    if (count > 1) {
        sd_send_cmd(CMD12, 0);
        sd_wait_card_busyflag();
    }
    sd_unselect();

    return res;
}

/**
 * @brief Write buffer data to SD card block
 * @param buff write buffer to store datas
 * @param sector Sector addres
 * @param count Sector numbers
 * @return RES_OK(0):success RES_ERROR(1):failure
 */
int MMC_disk_write(const uint8_t *buff, uint32_t sector, uint32_t count)
{
    uint8_t token;
    uint8_t *ptr = (uint8_t *)buff;
    
    // Fatfs block size 512bytes
    if (!(sd_card_type & CT_BLOCK))
        sector *= FATFS_CHUNK_SIZE;
    // Single or multiple block write token value
    token = (count == 1) ? 0xFE : 0xFC;

    sd_select();
    if (count == 1) {
        if ((sd_send_cmd(CMD24, sector) == 0) && (sd_write(ptr, token) == 0))
            count = 0;
        sd_wait_card_busyflag();
    } else {
        if (sd_card_type & CT_SDC)
            sd_send_cmd(ACMD23, count);
        if (sd_send_cmd(CMD25, sector) == 0) {
            do {
                if (sd_write(ptr, token) != 0)
                    break;
                sd_wait_card_busyflag();
                ptr += FATFS_CHUNK_SIZE;
            } while (--count);
            // Stop transfer date token(0xFD)
            if (sd_write(0, 0xFD) != 0)
                count = 1;
        }
    }
    sd_unselect();

    return (count == 0) ? RES_OK : RES_ERROR;
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
    uint8_t dummy = 0xFF;
    uint8_t n = 0;
    uint32_t csize = 0;

    sd_select();
    switch (cmd) {
    case CTRL_SYNC:
        // Send dummy clock force MOSI goes high
        HAL_SPI_Transmit(&hspi_sd, &dummy, 1, 0xFF);
        if (sd_wait_card_busyflag() == 0) {
            res = RES_OK;
        }
        break;
    case GET_SECTOR_COUNT:
        if ((sd_send_cmd(CMD9, 0) == 0) && (sd_read(csd, 16) == 0)) {
            if ((csd[0] >> 6) != 1) {
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((uint32_t)csd[7] << 2) + ((uint32_t)(csd[6] & 3) << 10) + 1;
                *(uint32_t *)buff = csize << (n - 9);
            } else {
                csize = csd[9] + ((uint32_t)csd[8] << 8) + ((uint32_t)(csd[7] & 63) << 16) + 1;
                *(uint32_t *)buff = csize << 10;
            }
            res = RES_OK;
        }
        break;

    case GET_BLOCK_SIZE:
        if (sd_card_type & CT_SDv2) {
            HAL_SPI_Transmit(&hspi_sd, &dummy, 1, 0xFF);
            if (sd_read(csd, 16) == 0) {
                // Discard trailing datas
                for (uint8_t i = 0; i < 48; ++i)
                    HAL_SPI_Transmit(&hspi_sd, &dummy, 1, 0xFF);
                res = RES_OK;
            }
        } else {
            if ((sd_send_cmd(CMD9, 0) == 0) && (sd_read(csd, 16) == 0)) {
                if (sd_card_type & CT_SDv1) {
                    *(uint32_t *)buff = (((csd[10] & 63) << 1) + ((uint32_t)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                } else {
                    *(uint32_t *)buff = ((uint32_t)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                }
                res = RES_OK;
            }
        }
        break;
    
    default:
        res = RES_PARERR;
        break;
    }
    sd_unselect();

    return res;
}

/**
 * @brief SPI transmit data done callback function
*/
void sd_spi_dmatx_complete_cb(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi_sd) {
#if USING_FREERTOS == 1
        BaseType_t task_need_woken;
        xSemaphoreGiveFromISR(dmatx_sync_sem, &task_need_woken);
#else
        dmatx_sync_var = 1;
#endif
    }
}

/**
 * @brief SPI receive data done callback function
*/
void sd_spi_dmatxrx_complete_cb(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi_sd) {
#if USING_FREERTOS == 1
        BaseType_t task_need_woken;
        xSemaphoreGiveFromISR(dmatxrx_sync_sem, &task_need_woken);
#else
        dmatxrx_sync_var = 1;
#endif
    }
}

/**
 * @brief SPI1 interrupt handle function
*/
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi_sd);
}

/**
 * @brief DMA1 channel2 interrupt handle function
*/
void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sd_txrx);
}

/**
 * @brief DMA1 channel3 interrupt handle function
*/
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sd_tx);
}
