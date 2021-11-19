#ifndef _SDMMC_H_
#define _SDMMC_H_

#ifdef _cplusplus
extern "C" {
#endif

void sdmmc_spi_init(void);
void sdmmc_cs_select(void);
void sdmmc_cs_unselect(void);

/*FatFs diskio layer MMC API interfaces*/
int MMC_disk_initialize(void);
int MMC_disk_status(void);
int MMC_disk_read(uint8_t *buff, uint32_t sector, uint32_t count);
int MMC_disk_write(const uint8_t *buff, uint32_t sector, uint32_t count);
int MMC_disk_ioctl(uint8_t cmd, void *buff);

#ifdef _cplusplus
}
#endif

#endif
