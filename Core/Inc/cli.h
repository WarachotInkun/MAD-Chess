/*
 * cli.h
 *
 *  Created on: Sep 29, 2025
 *      Author: kim00
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_



#endif /* INC_CLI_H_ */
#pragma once
#include <stddef.h>
int  uart_read_line(char *buf, size_t maxlen);
void io_puts(const char *s);
int  io_printf(const char *fmt, ...);

int  cli_parse_prefixed_coords(const char *line, char cmd, int *x, int *y);
