/*
 * File_Handling_RTOS.h
 *
 *  Created on: 14-May-2020
 *      Author: Controllerstech
 */

#ifndef FILE_HANDLING_RTOS_H_
#define FILE_HANDLING_RTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "fatfs.h"
#include "main.h"


/* mounts the sd card*/
uint8_t Mount_SD (const TCHAR* path);

/* unmounts the sd card*/
void Unmount_SD (const TCHAR* path);

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_SD (char* pat);

/* Only supports removing files from home directory. Directory remover to be added soon */
FRESULT Format_SD (void);

/* write the data to the file
 * @ name : is the path to the file*/
FRESULT Write_File (char *name, char *data, uint32_t num_bytes);

uint32_t entry_number_update(void);
uint8_t radio_log_exists(void);
FRESULT Update_File_u32 (char *name, uint32_t *data, uint32_t num_bytes);
FRESULT Write_File_16 (char *name, volatile int16_t *data, uint32_t num_bytes);
FRESULT Write_File_u16 (char *name, uint16_t *data, uint32_t num_bytes);
FRESULT Write_File_u8 (char *name, uint8_t *data, uint32_t num_bytes);

/* read data from the file
 * @ name : is the path to the file*/
FRESULT Read_File (char *name);

/* creates the file, if it does not exists
 * @ name : is the path to the file*/
FRESULT Create_File (char *name);

/* Removes the file from the sd card
 * @ name : is the path to the file*/
FRESULT Remove_File (char *name);

/* creates a directory
 * @ name: is the path to the directory
 */
FRESULT Create_Dir (char *name);

/* checks the free space in the sd card*/
uint32_t Check_SD_Space (void);

/* updates the file. write pointer is set to the end of the file
 * @ name : is the path to the file
 */
FRESULT Update_File (char *name, char *data);
FRESULT Update_File_16 (char *name, volatile int16_t *data, uint32_t num_bytes);
FRESULT Update_File_u8 (char *name, uint8_t *data, uint32_t num_bytes);


#ifdef __cplusplus
}
#endif

#endif /* FILE_HANDLING_RTOS_H_ */
