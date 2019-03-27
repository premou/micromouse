/*
 * configuration.h
 *
 *  Created on: 27 mars 2019
 *      Author: Invite
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string.h>

enum
{
	XKP = 0,
	XKI,
	XKD,
	WKP,
	WKI,
	WKD,
	XSPEED_LEARNING,
	WSPEED_LEARNING,
	WT1,
	WT2,
	WUT1,
	WUT2,
	WD,
	// TODO : add new configuration data
	CONFIGURATION_COUNT
};

void configuration_init();
void configuration_load();
void configuration_save();
float configuration_get_ptr(size_t index);

void configuration_parse_cli(char in);

#endif /* CONFIGURATION_H_ */
