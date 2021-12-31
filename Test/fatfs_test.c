#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "proj_config.h"

void fatfs_test_task(void *arg)
{
    (void)arg;

#if 1
    uint8_t i;
    FATFS fs;
    DIR dir;
    FIL fp;
    uint32_t res;
    uint32_t written, readcount;
    const char *path = "1:/DOC";
    const char *rwpath = "1:/DOC/test_fatfs.txt";
    const char *create_file_path = "1:/DOC/create_test.txt";
    const char wbuf[] = "The key to a successful open "
        "technology project is to ensure a neutral playing "
        "field for all developers, technologists, and companies "
        "to collectively contribute to project evolution and growth. "
        "The Linux Foundation was built on the idea of the democratization"
        "of code and scaling adoption, for all projects equally. Expert "
        "legal and governance support programs ensure everyone is on the "
        "same playing field.\n";
    char rbuf[1024];

    if ((res = f_mount(&fs, "1:/", 0)) == FR_OK) {
        if ((res = f_opendir(&dir, path)) == FR_OK) {
            static FILINFO fno;

            do {
                if ((res = f_readdir(&dir, &fno)) != FR_OK) {
                    printf("RES:%d f_readdir error!\r\n", res);
                    break;
                } else if (fno.fname[0] != '\0') {
                    printf("%s\r\n", fno.fname);
                }
            } while (fno.fname[0]);
        } else {
            printf("RES:%d f_opendir error!\r\n", res);
        }
        f_closedir(&dir);
        // Write data (mode a)
        res = f_open(&fp, rwpath, FA_WRITE | FA_OPEN_APPEND);
        if (res != FR_OK) {
            printf("RES:%d f_open to write error!\r\n", res);
        } else {
            if ((res = f_write(&fp, wbuf, sizeof(wbuf), &written)) != FR_OK)
                printf("RES:%d f_write error!\r\n", res);
            else
                printf("f_write[%d] Written[%d] success!\r\n", sizeof(wbuf), written);

            f_close(&fp);
        }

        // Read data
        res = f_open(&fp, rwpath, FA_READ);
        if (res != FR_OK) {
            printf("RES:%d f_open to read error!\r\n", res);
        } else {
            if ((res = f_read(&fp, rbuf, 1024, &readcount)) != FR_OK)
                printf("RES:%d f_read error!\r\n", res);
            else {
                printf("f_read[1024] Readden[%d] success!\r\n", readcount);
                rbuf[1023] = '\0';
                printf("%s", rbuf);
            }
            f_close(&fp);
        }

        // Create file
        res = f_open(&fp, create_file_path, FA_WRITE | FA_OPEN_ALWAYS);
        if (res != FR_OK) {
            printf("RES:%d f_open to create file error!\r\n", res);
        } else {
            if ((res = f_write(&fp, wbuf, sizeof(wbuf), &written)) != FR_OK)
                printf("RES:%d f_write error!\r\n", res);
            else
                printf("f_write[%d] Written[%d] success!\r\n", sizeof(wbuf), written);

            f_close(&fp);
        }
    } else {
        printf("RES:%d f_mount error!\r\n", res);
    }

    f_unmount("1:/");   
#else
    // Test read wifi firmware bin file 43362A2.bin file size 213732bytes
    static FATFS fs;
    FIL fp;
    const char *root_path = "1:/";
    const char *wifi_fw_path = "1:/assets/wifi/43362A2.bin";
    uint32_t res;
    const uint16_t read_chunk = 2048;
    uint8_t rbuf[read_chunk];
    uint8_t count = 0;
    uint32_t read_count;
    uint32_t total_size = 0;

    if ((res = f_mount(&fs, root_path, 1)) != FR_OK) {
        printf("RES[%d] f_mount failure!\r\n", res);
    } else {
        if ((res = f_open(&fp, wifi_fw_path, FA_READ)) != FR_OK) {
            printf("RES[%d] f_open %s failure!\r\n", res, wifi_fw_path);
        } else {
            puts("Start read...");
            do {
                if ((res = f_read(&fp, rbuf, read_chunk, &read_count)) != FR_OK) {
                    printf("RES[%d] count[%d] f_read failure!\r\n", ++count, res);
                    break;
                } else {
                    total_size += read_count;
                } 
            } while (read_chunk == read_count);
            puts("End read...");
            puts("wifi frameware total size 213732 bytes");
            printf("Read total size %d\r\n", total_size);
            printf("Last time read contents count[%d]:\r\n", read_count);
            // Out put last time read 50 bytes
            if (read_count >= 50) {
                for (uint8_t i = 0; i < 50; ++i) {
                    if ((i+1) % 10)
                        printf("%02x ", rbuf[read_count-50+i]);
                    else
                        printf("%02x\r\n", rbuf[read_count-50+i]);
                }
            } else {
                for (uint8_t i = 0; i < read_count; ++i) {
                    if ((i+1) % 10)
                        printf("%02x ", rbuf[i]);
                    else
                        printf("%02x\r\n", rbuf[i]);
                }
            }

            f_close(&fp);
        }
        f_unmount(root_path);
    }
#endif
    for (;;) {
        portDelayMs(10000);
    }
} 
