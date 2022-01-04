/*
 * File_Handling_RTOS.c
 *
 *  Created on: 14-May-2020
 *      Author: Controllerstech
 */

#include <File_Handling.h>
#include "stm32l4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================>>>>>>>> NO CHANGES AFTER THIS LINE =====================================>>>>>>> */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;



uint8_t Mount_SD (const TCHAR* path)
{
	fresult = f_mount(&fs, path, 1);
	return fresult;
}

void Unmount_SD (const TCHAR* path)
{
	fresult = f_mount(NULL, path, 1);
}

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_SD (char* pat)
{
    DIR dir;
    UINT i;
    char *path = malloc(20*sizeof (char));
    sprintf (path, "%s",pat);

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	char *buf = malloc(30*sizeof(char));
            	sprintf (buf, "Dir: %s\r\n", fno.fname);
            	println(buf);
            	free(buf);
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                fresult = Scan_SD(path);                     /* Enter the directory */
                if (fresult != FR_OK) break;
                path[i] = 0;
            }
            else
            {   /* It is a file. */
           	   char *buf = malloc(30*sizeof(char));
               sprintf(buf,"File: %s/%s\n", path, fno.fname);
               println(buf);
               free(buf);
            }
        }
        f_closedir(&dir);
    }
    free(path);
    return fresult;
}

/* Only supports removing files from home directory */
FRESULT Format_SD (void)
{
    DIR dir;
    char *path = malloc(20*sizeof (char));
    sprintf (path, "%s","/");

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	fresult = f_unlink(fno.fname);
            	if (fresult == FR_DENIED) continue;
            }
            else
            {   /* It is a file. */
               fresult = f_unlink(fno.fname);
            }
        }
        f_closedir(&dir);
    }
    free(path);
    return fresult;
}




FRESULT Write_File (char *name, char *data, uint32_t num_bytes)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, num_bytes, &bw);

	    	/* Close file */
	    	fresult = f_close(&fil);
	    }
	    return fresult;
	}
}


FRESULT Write_File_u8 (char *name, uint8_t *data, uint32_t num_bytes)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, num_bytes, &bw);

	    	/* Close file */
	    	fresult = f_close(&fil);
	    return fresult;
	    }
	}
}


FRESULT Write_File_u16 (char *name, uint16_t *data, uint32_t num_bytes)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, num_bytes, &bw);

	    	/* Close file */
	    	fresult = f_close(&fil);
	    return fresult;
	    }
	}
}

FRESULT Write_File_16 (char *name, volatile int16_t *data, uint32_t num_bytes)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, (int16_t*)data, num_bytes, &bw);

	    	/* Close file */
	    	fresult = f_close(&fil);
	    }
	    return fresult;
	}
}

uint32_t entry_number_update(void)
{
	uint32_t entry = 0;
	char *name = "ENTRY.bin";

/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		//Create the file and populate it with binary value 1
		println("Creating ENTRY.bin");
		Create_File(name);
		char entry_str[100];
		entry = 2;
		sprintf(entry_str, "%lu", entry);
		Update_File(name, entry_str);
		return 1;
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
			return 0;
		}

		/* Read data from the file
		* see the function details for the arguments */
		char *buffer = malloc(sizeof(f_size(&fil)));
		UINT numberBytes[1] = {(UINT)4};
		fresult = f_read (&fil, buffer, f_size(&fil), numberBytes);
		if (fresult != FR_OK)
		{
			return 0;
		}

		else
		{
			entry = atoi(buffer);
		}

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
			return 0;
		}

		Remove_File(name);
		Create_File(name);

		char entry_str[100] = {0};
		sprintf(entry_str, "%luXXX", (entry + 1));	//the XXX seems to add characters and stop atoi reading extra numbers
		Update_File(name, entry_str);

		return entry;
	}
	return 0;
}

uint8_t radio_log_exists(void)
{
	fresult = f_stat ("RADIO.TXT", &fno);
	if (fresult != FR_OK) return 0;
	return 1;
}

FRESULT Read_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
		    return fresult;
		}

		/* Read data from the file
		* see the function details for the arguments */

		char *buffer = malloc(sizeof(f_size(&fil)));
		fresult = f_read (&fil, buffer, f_size(&fil), &br);
		if (fresult == FR_OK)
		{
			free(buffer);

			/* Close file */
			fresult = f_close(&fil);
		}
	    return fresult;
	}
}

FRESULT Create_File (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
	    return fresult;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
		if (fresult != FR_OK)
		{
		    return fresult;
		}

		fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File (char *name, char *data)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing text */
	    fresult = f_write(&fil, data, strlen (data), &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File_16 (char *name, volatile int16_t *data, uint32_t num_bytes)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing data */
	    fresult = f_write(&fil, (int16_t*)data, num_bytes, &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File_u8 (char *name, uint8_t *data, uint32_t num_bytes)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing data */
	    fresult = f_write(&fil, (int16_t*)data, num_bytes, &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File_u32 (char *name, uint32_t *data, uint32_t num_bytes)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
	    return fresult;
	}

	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	        return fresult;
	    }

	    /* Writing data */
	    fresult = f_write(&fil, (int16_t*)data, num_bytes, &bw);

	    /* Close file */
	    fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Remove_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		return fresult;
	}

	else
	{
		fresult = f_unlink (name);
	}
	return fresult;
}

FRESULT Create_Dir (char *name)
{
    fresult = f_mkdir(name);
    return fresult;
}

uint32_t Check_SD_Space (void)
{
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);
    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    return free_space;
}

#ifdef __cplusplus
}
#endif

