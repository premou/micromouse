/*
 * configuration.h
 *
 *  Created on: 27 mars 2019
 *      Author: Invite
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string.h>

// Remove comment to read command from serial link
//#define READ_SERIAL_LINK

// Macro
// =-=-=
#define COUNT(x)                ((int)(sizeof(x)/sizeof(*(x))))
#define CRC32_SIZE              (sizeof(uint32_t))
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

// Prototypes
// =-=-=-=-=-
void configuration_init();
void configuration_save();
void configuration_parse_cli(char in);

#endif /* CONFIGURATION_H_ */
