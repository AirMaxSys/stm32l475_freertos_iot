#ifndef _SDMMC_H_
#define _SDMMC_H_

#ifdef _cplusplus
extern "C" {
#endif

typedef unsigned long DWORD;

void sdmmc_spi_init(void);
void sdmmc_cs_select(void);
void sdmmc_cs_unselect(void);

#ifdef _cplusplus
}
#endif

#endif
