/*
 * configuration.c
 *
 *  Created on: 27 mars 2019
 *      Author: Invite
 */

// Includes
// =-=-=-=-
#include "configuration.h"
#include "serial.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Extern for serial access
// =-=-=-=-=-=-=-=-=-=-=-=-
extern HAL_Serial_Handler com;

// Parameters txt: * SIZE MUST BE EQUAL TO 8 CHAR *
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const char *parameters_txt[] = {
	"ID_____0",
    "ID_____1",
    "ID_____2",
    "CRC32___" // <<<<= CRC should always be at the end
};

// Parameters in FLASH
// =-=-=-=-=-=-=-=-=-=
char   name_from_flash[] = {
  "_______\0"
};

double parameters_from_flash[] = {
  /*   0 */		0.0,
  /*   1 */		0.0,
  /*   2 */		0.0,
};

// Define
// =-=-=-
#define NB_MAX_PARAM             (COUNT(parameters_from_flash) + 1)
#define PARAMETERS_SIZE          (sizeof(parameters_from_flash))
#define NAME_SIZE                (sizeof(name_from_flash))
#define CRC32_INDEX              (NB_MAX_PARAM - 1)
#define COMMAND_MAX_SIZE         (1 * 1024)
#define CONFIGURATION_FLASH_ADDR ((uint32_t*)(0x080E0000))
#define NAME_FLASH_ADDR          (CONFIGURATION_FLASH_ADDR)
#define PARAMETERS_FLASH_ADDR    (NAME_FLASH_ADDR + NAME_SIZE)

// Const
// =-=-=
const unsigned char base64_table[65] =
"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Static
// =-=-=-
static char command[COMMAND_MAX_SIZE];

// Functions
// =-=-=-=-=

// CRC32
// =-=-=
// compute_crc32: compute the CRC32 of a buffer
unsigned int compute_crc32(unsigned char *message, uint32_t len)
{
    int i, j;
    unsigned int byte, crc, mask;

    i = 0;
    crc = 0xFFFFFFFF;
    while (len--)
    {
        byte = message[i];            // Get next byte.
        crc = crc ^ byte;
        for (j = 7; j >= 0; j--) {    // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
        i = i + 1;
    }
    return ~crc;
}

// BASE64
// =-=-=-

// base64_encode: as named
unsigned char * base64_encode(unsigned char *src,
                              uint32_t len,
                              uint32_t *out_len)
{
    unsigned char *out, *pos;
    const unsigned char *end, *in;
    uint32_t olen;
    int line_len;

    olen = len * 4 / 3 + 4; /* 3-byte blocks to 4-byte */
    olen += olen / 72; /* line feeds */
    olen++; /* nul termination */
    if (olen < len)
        return NULL; /* integer overflow */
    out = malloc(olen);
    if (out == NULL)
        return NULL;

    end = src + len;
    in = src;
    pos = out;
    line_len = 0;
    while (end - in >= 3) {
        *pos++ = base64_table[in[0] >> 2];
        *pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
        *pos++ = base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
        *pos++ = base64_table[in[2] & 0x3f];
        in += 3;
        line_len += 4;
        if (line_len >= 72) {
            *pos++ = '\n';
            line_len = 0;
        }
    }

    if (end - in) {
        *pos++ = base64_table[in[0] >> 2];
        if (end - in == 1) {
            *pos++ = base64_table[(in[0] & 0x03) << 4];
            *pos++ = '=';
        } else {
            *pos++ = base64_table[((in[0] & 0x03) << 4) |
                                  (in[1] >> 4)];
            *pos++ = base64_table[(in[1] & 0x0f) << 2];
        }
        *pos++ = '=';
        line_len += 4;
    }

    if (line_len)
        *pos++ = '\n';

    *pos = '\0';
    if (out_len)
        *out_len = pos - out;
    return out;
}// end of base64_encode

// base64_decode: as named
unsigned char * base64_decode(unsigned char *src,
                              uint32_t len,
                              uint32_t *out_len)
{
    unsigned char dtable[256], *out, *pos, block[4], tmp;
    size_t i, count, olen;
    int pad = 0;

    memset(dtable, 0x80, 256);
    for (i = 0; i < sizeof(base64_table) - 1; i++)
        dtable[base64_table[i]] = (unsigned char) i;
    dtable['='] = 0;

    count = 0;
    for (i = 0; i < len; i++) {
        if (dtable[src[i]] != 0x80)
            count++;
    }

    if (count == 0 || count % 4)
        return NULL;

    olen = count / 4 * 3;
    pos = out = malloc(olen);
    if (out == NULL)
        return NULL;

    count = 0;
    for (i = 0; i < len; i++) {
        tmp = dtable[src[i]];
        if (tmp == 0x80)
            continue;

        if (src[i] == '=')
            pad++;
        block[count] = tmp;
        count++;
        if (count == 4) {
            *pos++ = (block[0] << 2) | (block[1] >> 4);
            *pos++ = (block[1] << 4) | (block[2] >> 2);
            *pos++ = (block[2] << 6) | block[3];
            count = 0;
            if (pad) {
                if (pad == 1)
                    pos--;
                else if (pad == 2)
                    pos -= 2;
                else {
                    /* Invalid padding */
                    free(out);
                    return NULL;
                }
                break;
            }
        }
    }

    *out_len = pos - out;
    return out;
}// end of base64_decode

// Configuration
// =-=-=-=-=-=-=
void configuration_init()
{
    // Pre-compilation test
    BUILD_BUG_ON(COUNT(parameters_from_flash) != (COUNT(parameters_txt) - 1));

    // Copy flash to ram
    memcpy((void *)name_from_flash, (void *)NAME_FLASH_ADDR, NAME_SIZE);
    if(strnlen(name_from_flash, NAME_SIZE) != NAME_SIZE)
    {
    	memcpy((void *)name_from_flash, (void *)"UNKNOWN\0", NAME_SIZE);
    }
    name_from_flash[NAME_SIZE - 1] = '\0';
    memcpy((void *)parameters_from_flash, (void *)PARAMETERS_FLASH_ADDR, PARAMETERS_SIZE);
}

void configuration_parse_cli(char in)
{
	static size_t  position = 0   ;
	int            p              ;
	uint32_t       crc32_encode   ;
	uint32_t       crc32_decode   ;
    unsigned char *pEncodedBase64 ;
    unsigned char *pDecodedBase64 ;
    uint32_t       length_encode  ;
    uint32_t       length_decode  ;
    unsigned char *pData          ;

	// 0) fill buffer
    // =-=-=-=-=-=-=-
	command[position++] = in;

	// 1) end of line or full
	// =-=-=-=-=-=-=-=-=-=-=-
	if( (position==COMMAND_MAX_SIZE-1) || (in=='\n') )
	{
		command[position] = 0; // null terminated string
		char * token = strtok(command," \n\r");

		// 3) parse the command
		// =-=-=-=-=-=-=-=-=-=-

		// =-=-=-=
		// get_api
		// =-=-=-=
		if(strcmp(token,"get_api")==0)
		{
			configuration_init();

			// Response: api [number of parameters] [list of text parameters]
			HAL_Serial_Print(&com,"configuration get_api %d ", NB_MAX_PARAM);
			for(p=0 ; p<NB_MAX_PARAM ; p++)
			{
				HAL_Serial_Print(&com,"%s ", parameters_txt[p]);
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		// =-=-=-=
		// get_nam
		// =-=-=-=
		if(strcmp(token,"get_nam")==0)
		{
			HAL_Serial_Print(&com,"configuration get_nam %s\r\n", name_from_flash);
		}
		// =-=-=-=
		// get_all
		// =-=-=-=
		else if(strcmp(token,"get_all")==0)
		{
			// a) compute CRC32
		    crc32_encode = compute_crc32((unsigned char *)&parameters_from_flash,
		    			        		  PARAMETERS_SIZE);

		    pData = malloc(PARAMETERS_SIZE + CRC32_SIZE);
		    memcpy(pData, (void *)&parameters_from_flash, PARAMETERS_SIZE);
		    memcpy(pData + PARAMETERS_SIZE, (void *)&crc32_encode, CRC32_SIZE);

		    // b) build base64 message
		    length_encode = 0;
		    pEncodedBase64 = base64_encode((unsigned char *)pData,
		    		                        PARAMETERS_SIZE + CRC32_SIZE,
		                                    &length_encode);
		    if((NULL != pEncodedBase64) && (length_encode < COMMAND_MAX_SIZE))
		    {
		    	// Response: all [base64 message]
		    	HAL_Serial_Print(&com,"configuration get_all %s\r\n", pEncodedBase64);
		    }
		    else
		    {
		    	HAL_Serial_Print(&com,"ERROR get_all command (0x%x %d)\r\n", pEncodedBase64, length_encode);
		    }
		    if(pEncodedBase64 != NULL)
		    {
		    	free(pEncodedBase64);
		    }
		}
		// =-=-=-=
		// set_all
		// =-=-=-=
		else if(strcmp(token,"set_all")==0)
		{
			token = strtok(NULL," \r\n");
			if(token != NULL)
			{
				// a) decode base64
				length_decode = 0;
				length_encode = strlen(token);
				pDecodedBase64 = base64_decode((unsigned char *)token,
			                                   length_encode,
			                                   &length_decode);
			    if((pDecodedBase64 != NULL) && ((PARAMETERS_SIZE + CRC32_SIZE) == length_decode))
			    {
			    	// b) compute crc32
			    	crc32_decode = compute_crc32((unsigned char *)pDecodedBase64,
			    			                     PARAMETERS_SIZE);
			    	memcpy((void *)&crc32_encode,
			    		   ((unsigned char *)pDecodedBase64) + PARAMETERS_SIZE,
						   CRC32_SIZE);

			    	// c) check crc32
			    	if(crc32_decode == crc32_encode)
			    	{
			            memcpy((void *)parameters_from_flash, (void *)pDecodedBase64, PARAMETERS_SIZE);
			            HAL_Serial_Print(&com,"configuration set_all CRC32 OK\n");
			    	}
			    	else
			    	{
				    	HAL_Serial_Print(&com,"ERROR set_all command bad crc (0x%x 0x%x)\r\n",
				    					 crc32_decode, crc32_encode);
			    	}
			    	free(pDecodedBase64);
			    }
			    else
			    {
			    	HAL_Serial_Print(&com,"ERROR set_all base64decode KO\r\n");
			    }

			}

			// name
			token = strtok(NULL," \r\n");
			if(token != NULL)
			{
				length_encode = strnlen(token, NAME_SIZE);
				memcpy((void *)name_from_flash, (void *)token, NAME_SIZE);
				name_from_flash[NAME_SIZE - 1] = '\0';
			}
		}
		// =-=-=-=
		// sav_all
		// =-=-=-=
		else if(strcmp(token,"sav_all")==0)
		{
			configuration_save_to_flash();
			HAL_Serial_Print(&com,"sav_all OK\r\n");
		}
		// =-=-=-=-=-=-=-=
		// Unknown command
		// =-=-=-=-=-=-=-=
		else
		{
			HAL_Serial_Print(&com,"ERROR UNKNOWN command\r\n");
		}

		// 4) reset parser
		// =-=-=-=-=-=-=-=
		position = 0; // reset position
		memset(command,0,COMMAND_MAX_SIZE);
	}
}

void configuration_save_to_flash()
{
	uint32_t           size          ;
	uint32_t           flash_address ;
	uint32_t          *pData         ;
	HAL_StatusTypeDef  res           ;
	uint32_t           error         ;
	uint32_t           index         ;

	if(HAL_OK != HAL_FLASH_Unlock())
	{
		HAL_Serial_Print(&com,"ERROR sav_all unlock\r\n");
		HAL_FLASH_Lock();
		return;
	}

	// erasing
	// =-=-=-=
	FLASH_EraseInitTypeDef erase = {
			FLASH_TYPEERASE_SECTORS,
			FLASH_BANK_1,
			FLASH_SECTOR_7,
			1,
			FLASH_VOLTAGE_RANGE_3
	};

	error = 0;
	HAL_FLASHEx_Erase(&erase, &error);
	if(error!=0xFFFFFFFF)
	{
		HAL_Serial_Print(&com,"ERROR sav_all erase returns %d\r\n", error);
		HAL_FLASH_Lock();
		return;
	}

	// programming into flash
	// =-=-=-=-=-=-=-=-=-=-=-
	size          = NAME_SIZE / sizeof(uint32_t)      ;
	flash_address = (uint32_t)(NAME_FLASH_ADDR)       ;
	pData         = (uint32_t *)(&name_from_flash[0]) ;
	for(index=0; index < size; ++index)
	{
		res  = HAL_ERROR;
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (uint64_t)(*pData));
		if(res != HAL_OK)
		{
			HAL_Serial_Print(&com,"ERROR sav_all write returns %d\r\n", res);
			HAL_FLASH_Lock();
			return;
		}
		flash_address += sizeof(uint32_t) ;
		pData++;
	}

	size          = PARAMETERS_SIZE / sizeof(uint32_t)      ;
	flash_address = (uint32_t)(PARAMETERS_FLASH_ADDR)       ;
	pData         = (uint32_t *)(&parameters_from_flash[0]) ;
	for(index=0; index < size; ++index)
	{
		res  = HAL_ERROR;
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, (uint64_t)(*pData));
		if(res != HAL_OK)
		{
			HAL_Serial_Print(&com,"ERROR sav_all write returns %d\r\n", res);
			HAL_FLASH_Lock();
			return;
		}
		flash_address += sizeof(uint32_t) ;
		pData++;
	}

	HAL_FLASH_Lock();
}

// End of file configuration
