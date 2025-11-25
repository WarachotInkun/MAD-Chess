/*
 * cli.c
 *
 *  Created on: Sep 29, 2025
 *      Author: kim00
 */

#include "cli.h"
#include "usart.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

int uart_read_line(char *buf, size_t maxlen){
    size_t i = 0;
    for(;;){
        uint8_t c;
        if (HAL_UART_Receive(&huart3, &c, 1, HAL_MAX_DELAY) != HAL_OK) return -1;
        if (c=='\r'||c=='\n'){ HAL_UART_Transmit(&huart3,(uint8_t*)"\r\n",2,100); break; }
        if ((c==0x08||c==0x7F) && i>0){ i--; HAL_UART_Transmit(&huart3,(uint8_t*)"\b \b",3,100); continue; }
        if (i+1<maxlen){ buf[i++]=(char)c; HAL_UART_Transmit(&huart3,&c,1,100); }
    }
    buf[i]='\0';
    return (int)i;
}
void io_puts(const char *s){ HAL_UART_Transmit(&huart3,(uint8_t*)s,strlen(s),100); }

int  io_printf(const char *fmt, ...){
    char b[64]; va_list ap; va_start(ap,fmt);
    int n = vsnprintf(b,sizeof(b),fmt,ap); va_end(ap);
    if(n>0) HAL_UART_Transmit(&huart3,(uint8_t*)b,(uint16_t)n,100);
    return n;
}

int cli_parse_prefixed_coords(const char *line, char cmd, int *x, int *y) {
    if (!line || !x || !y) return 0;
    /* ข้ามช่องว่างต้นบรรทัด */
    while (*line==' ' || *line=='\t') line++;

    if (tolower((unsigned char)line[0]) != tolower((unsigned char)cmd)) return 0;
    line++; /* ข้ามอักษรคำสั่ง เช่น 'd' */

    /* ข้ามตัวคั่นที่อาจมี */
    while (*line==' ' || *line=='\t' || *line==':') line++;

    /* รองรับทั้ง "x,y" และ "x y" */
    if (sscanf(line, "%d , %d", x, y) == 2) return 1;
    if (sscanf(line, "%d %d",   x, y) == 2) return 1;

    return 0;
}
