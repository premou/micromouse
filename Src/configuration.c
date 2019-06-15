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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Extern for serial access
// =-=-=-=-=-=-=-=-=-=-=-=-
extern HAL_Serial_Handler com;

// Parameters txt: * SIZE MUST BE EQUAL TO 8 CHAR *
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const char *parameters_txt[] = {
//  "12345678"
    "ID_____0",
    "ID_____1",
    "ID_____2",

    "CRC___32" // <<<<= CRC should always be at the end
};

// Parameters in FLASH
// =-=-=-=-=-=-=-=-=-=
float parameters_in_flash[] = {
    /*   0 */		100.0,
	/*   1 */		0.3,
	/*   2 */		12.0,
};

// Parameters in RAM
// =-=-=-=-=-=-=-=-=
uint32_t parameters_in_ram[] = {
    /*   0 */		0,
	/*   1 */		0,
	/*   2 */		0,

    /* CRC */		0  // <<<<= MUST BE THE LAST ENTRY
};

// Define
// =-=-=-
#define NB_MAX_PARAM            COUNT(parameters_in_ram)
#define CRC32_INDEX             (NB_MAX_PARAM - 1)
#define COMMAND_MAX_SIZE        (1 * 1024)

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
    // Pre compilation test
    BUILD_BUG_ON(COUNT(parameters_in_ram) != COUNT(parameters_txt));

	// Read parameters from flash and (1) copy in ram & (2) convert in uint_32
	for(int p=0 ; p<(NB_MAX_PARAM - 1 /* without CRC32 */) ; p++)
	{
		parameters_in_ram[p] = (uint32_t) (parameters_in_flash[p] * 1000.0);
	}
}

void configuration_save()
{
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
			// Copy the parameters from flash to ram
			configuration_init();

			// Response: api [nomber of parameters] [list of text parameters]
			HAL_Serial_Print(&com,"configuration get_api %d ", NB_MAX_PARAM);
			for(p=0 ; p<NB_MAX_PARAM ; p++)
			{
				HAL_Serial_Print(&com,"%s ", parameters_txt[p]);
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		// =-=-=-=
		// get_all
		// =-=-=-=
		else if(strcmp(token,"get_all")==0)
		{
			// a) compute CRC32
		    crc32_encode = compute_crc32((unsigned char *)&parameters_in_ram,
		    			        		  sizeof(parameters_in_ram) - CRC32_SIZE);
		    parameters_in_ram[CRC32_INDEX] = crc32_encode ;

		    // b) build base64 message
		    length_encode = 0;
		    pEncodedBase64 = base64_encode((unsigned char *)&parameters_in_ram,
		                                    sizeof(parameters_in_ram),
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
		// =-=-=-=-
		// show_all
		// =-=-=-=-
		else if(strcmp(token,"show_all")==0)
		{
			// Response: display all the parameter in int format
			HAL_Serial_Print(&com,"configuration show_all %d parameters\n", NB_MAX_PARAM);
			for(p=0 ; p<NB_MAX_PARAM ; p++)
			{
				HAL_Serial_Print(&com,"# %s => %d (0x%x)\n",
						         parameters_txt[p], parameters_in_ram[p], parameters_in_ram[p]);
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
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
			    if((pDecodedBase64 != NULL) && (sizeof(parameters_in_ram) == length_decode))
			    {
			    	// b) compute and compare crc32
			    	crc32_decode = compute_crc32((unsigned char *)pDecodedBase64,
	    					                     sizeof(parameters_in_ram) - CRC32_SIZE);
			    	memcpy((void *)&crc32_encode,
			    		   ((unsigned char *)pDecodedBase64) + sizeof(parameters_in_ram) - CRC32_SIZE,
						   CRC32_SIZE);

			    	// c) check crc32
			    	if(crc32_decode == crc32_encode)
			    	{
			            memcpy((void *)parameters_in_ram, (void *)pDecodedBase64, length_decode);
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

#if 0
void configuration_parse_cli(char in)
{
	#define COMMAND_MAX_SIZE 256
	static char command[COMMAND_MAX_SIZE];
	static size_t position = 0;
	// 0) fill buffer
	command[position++] = in;
	// 1) end of line or full
	if( (position==COMMAND_MAX_SIZE-1) || (in=='\n') )
	{
		command[position] = 0; // null terminated string
		char * token = strtok(command," \n\r");
		if(strcmp(token,"get")==0)
		{
			HAL_Serial_Print(&com,"get ");
			for(size_t index=0;index<CONFIGURATION_COUNT;++index)
			{
				HAL_Serial_Print(&com,"%d ",(int32_t)(config[index]*1000.0)); // apply x1000 factor to see decimals
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		else if(strcmp(token,"set")==0)
		{
			token = strtok(NULL," \r\n");
			size_t index = 0;
			while( token != NULL )
			{
				config[index++] = (float)(atoi(token))/1000.0;
				token = strtok(NULL," \r\n");
			}
			HAL_Serial_Print(&com,"\r\n  UDPATED to:");
			for(size_t index=0;index<CONFIGURATION_COUNT;++index)
			{
				HAL_Serial_Print(&com,"%d ",(int32_t)(config[index]*1000.0)); // apply x1000 factor to see decimals
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		else if(strcmp(token,"save")==0)
		{
			configuration_save();
			HAL_Serial_Print(&com,"\r\n  SAVED to:");
			for(size_t index=0;index<CONFIGURATION_COUNT;++index)
			{
				HAL_Serial_Print(&com,"%d ",(int32_t)(config[index]*1000.0)); // apply x1000 factor to see decimals
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		else if(strcmp(token,"load")==0)
		{
			configuration_load();
			HAL_Serial_Print(&com,"\r\n  LOADED to:");
			for(size_t index=0;index<CONFIGURATION_COUNT;++index)
			{
				HAL_Serial_Print(&com,"%d ",(int32_t)(config[index]*1000.0)); // apply x1000 factor to see decimals
				HAL_Delay(1);
			}
			HAL_Serial_Print(&com,"\r\n");
		}
		// reset parser
		position = 0; // reset position
		memset(command,0,COMMAND_MAX_SIZE);
	}
}
#endif

#if 0
#define CONFIGURATION_FLASH_ADDR ((uint32_t*)(0x080E0000))
void configuration_save_to_flash()
{
	HAL_FLASH_Unlock();

	// erasing
	FLASH_EraseInitTypeDef erase = {
			FLASH_TYPEERASE_SECTORS,
			FLASH_BANK_1,
			FLASH_SECTOR_7,
			1,
			FLASH_VOLTAGE_RANGE_3
	};
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase,&error);

	// programming 256-bit word into flash
	{
		uint32_t size_256bits = sizeof(configuration_data)/32;
		uint32_t flash_address = (uint32_t)(CONFIGURATION_FLASH_ADDR);
		uint64_t data_address = (uint64_t)( (uint64_t const *)(&config) );
		for(uint64_t index=0; index<size_256bits; ++index)
		{
			HAL_FLASH_Program( FLASH_TYPEPROGRAM_FLASHWORD, flash_address, data_address );
			flash_address += 32;
			data_address += 32;
		}
	}
	HAL_FLASH_Lock();
}

void configuration_load_from_flash()
{
	// read back
	uint32_t size_64bits = sizeof(configuration_data)/8;
	uint64_t * ptr = (uint64_t *)(&config);
	for(uint32_t index=0; index<size_64bits; ++index)
	{
		*(ptr+index) = *((uint64_t *)CONFIGURATION_FLASH_ADDR+index);
	}
}
#endif
