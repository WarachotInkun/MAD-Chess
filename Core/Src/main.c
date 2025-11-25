/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "Micro_Max.h"
#include <stdio.h>
#include "config.h"
//#include "cli.h"
#include "motion.h"
#include "homing.h"
#include "stepper.h"
#include <stdbool.h>
#include "tim.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"
#include <ctype.h>
#include <stdlib.h>
#include "core_cm7.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} MuxEnablePin;

MuxEnablePin mux_enables[4] = {
    {GPIOB, GPIO_PIN_8},   // MUX 0
    {GPIOB, GPIO_PIN_9},   // MUX 1
    {GPIOA, GPIO_PIN_5},   // MUX 2
    {GPIOA, GPIO_PIN_6},   // MUX 3
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_TIMEOUT    HAL_MAX_DELAY
#define LINE_BUFSIZE    64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t check = 0;

volatile uint32_t player1_time = 1800;
volatile uint32_t player2_time = 1800;
volatile uint8_t  current_player = 1;
uint8_t button1_state = 1;
uint8_t button15_state = 1;
uint8_t last_button_state1 = 1; // pull-up = idle
uint8_t last_button_state15 = 1;
uint8_t button2_state = 1;
uint8_t last_button_state2 = 1; // pull-up = idle


char formatted1[64];
uint8_t send_ready = 0;
uint8_t button1_flag = 0;
uint8_t button15_flag = 0;
uint8_t button2_flag = 0;
#define RX_BUFFER_SIZE 64
uint8_t rx_char;             // ตัวอักษรที่รับเข้าทีละตัว
char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_idx = 0;

bool mg = false;

uint32_t max_board[4][16] = {0};
uint32_t min_board[4][16] = {9999};

uint32_t prev_board[4][16] = {0};
uint32_t prev_threshold[4][16];
uint8_t total_piece = 32;
uint8_t count_piece = 0;
volatile bool curr_board0[4][16] = {0};
uint32_t curr_board[4][16] = {0};
char captured_square[3] = {0};
uint32_t board_empty[4][16];
uint32_t threshold[4][16];

bool whiteSide = true;
uint8_t started = 0;
const char *square_map[4][16] = {
    // mux 0  (was rank 8–7 → now rank 1–2)
    {
        "h1","g1","f1","e1","d1","c1","b1","a1",
        "h2","g2","f2","e2","d2","c2","b2","a2"
    },
    // mux 1  (was rank 6–5 → now rank 3–4)
    {
        "h3","g3","f3","e3","d3","c3","b3","a3",
        "h4","g4","f4","e4","d4","c4","b4","a4"
    },
    // mux 2  (was rank 4–3 → now rank 5–6)
    {
        "h5","g5","f5","e5","d5","c5","b5","a5",
        "h6","g6","f6","e6","d6","c6","b6","a6"
    },
    // mux 3  (was rank 2–1 → now rank 7–8)
    {
        "h7","g7","f7","e7","d7","c7","b7","a7",
        "h8","g8","f8","e8","d8","c8","b8","a8"
    }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char select_initial_string(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char mov[5] = {0};
static void uart_puts(const char *s) {
  HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), UART_TIMEOUT);
}

/* อ่าน 1 บรรทัด (จบเมื่อเจอ CR/LF), เก็บลง buf แบบ zero-terminated
   คืนค่าความยาวสตริง (ไม่รวม \0) */
static int uart_getline(char *buf, int maxlen) {
  int idx = 0;
  while (idx < maxlen - 1) {
    uint8_t ch;
    if (HAL_UART_Receive(&huart3, &ch, 1, UART_TIMEOUT) != HAL_OK) {
      continue; // or return -1 หากอยากให้หลุด
    }

    // echo (option) — ให้เห็นบน TeraTerm ว่าพิมพ์อะไร
    HAL_UART_Transmit(&huart3, &ch, 1, UART_TIMEOUT);

    if (ch == '\r') {
      // รอ LF ต่อ (ถ้ามี)
      uint8_t next;
      if (HAL_UART_Receive(&huart3, &next, 1, 5) == HAL_OK && next != '\n') {
        // ไม่ใช่ LF ก็เอาคืน buffer (ถ้าต้องการ)
      }
      break;
    } else if (ch == '\n') {
      break;
    } else if (ch == 0x08 || ch == 0x7F) {
      // Backspace
      if (idx > 0) {
        idx--;
      }
    } else {
      buf[idx++] = (char)ch;
    }
  }
  buf[idx] = '\0';
  return idx;
}

/* ตรวจรูปแบบ "e2e4": ไฟล์ a..h, แถว 1..8, 4 อักษรพอดี */
static int is_valid_mov4(const char *s) {
  return (strlen(s) == 4) &&
         (s[0] >= 'a' && s[0] <= 'h') &&
         (s[1] >= '1' && s[1] <= '8') &&
         (s[2] >= 'a' && s[2] <= 'h') &&
         (s[3] >= '1' && s[3] <= '8');
}

static int char_to_int(char s) {
  if(s == 'h' || s=='1'){return 0;}
  if(s == 'g' || s=='2'){return 1;}
  if(s == 'f' || s=='3'){return 2;}
  if(s == 'e' || s=='4'){return 3;}
  if(s == 'd' || s=='5'){return 4;}
  if(s == 'c' || s=='6'){return 5;}
  if(s == 'b' || s=='7'){return 6;}
  if(s == 'a' || s=='8'){return 7;}
}

static void set_mux_channel(uint8_t counter)
{
    // counter = 0..15
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,  (counter & (1<<0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // SEL0 (LSB)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, (counter & (1<<1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // SEL1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, (counter & (1<<2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // SEL2
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (counter & (1<<3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // SEL3 (MSB)
}


static void enable_mux(uint8_t mux_index) {
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(mux_enables[i].port, mux_enables[i].pin, GPIO_PIN_SET); // disable all (assuming active LOW)
    }
    HAL_GPIO_WritePin(mux_enables[mux_index].port, mux_enables[mux_index].pin, GPIO_PIN_RESET); // enable selected mux
}


void print_board(void) {
    char msg[40];
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

    for (int rank = 0; rank < 8; rank++) {
        for (int file = 0; file < 8; file++) {
            int index = rank * 8 + file; // 0..63
            int mux = index / 16;        // 0..3
            int ch  = index % 16;        // 0..15


            char c = (curr_board[mux][ch] < threshold[mux][ch]) ? '1' : '.';

            sprintf(msg, "%c ", c);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"_______________________________\r\n", strlen("_______________________________\r\n"), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
        for (int rank = 0; rank < 8; rank++) {
            for (int file = 0; file < 8; file++) {
                int index = rank * 8 + file; // 0..63
                int mux = index / 16;        // 0..3
                int ch  = index % 16;        // 0..15


                char c = (prev_board[mux][ch] < prev_threshold[mux][ch]) ? '1' : '.';

                sprintf(msg, "%c ", c);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
        }



    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

uint32_t read_adc(void) {
    uint32_t value = 0;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return value;
}

void check_knight_move(const char *from, const char *to) {
    if (!from || !to || strlen(from) < 2 || strlen(to) < 2)
        return;

    int from_file = toupper(from[0]) - 'a' + 1;
    int from_rank = from[1] - '0';
    int to_file   = toupper(to[0]) - 'a' + 1;
    int to_rank   = to[1] - '0';

    int dx = abs(to_file - from_file);
    int dy = abs(to_rank - from_rank);

    if ((dx == 2 && dy == 1) || (dx == 1 && dy == 2)) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "Knight move detected: %s -> %s\r\n", from, to);
        uart_puts(buffer);

        uart_puts("Squares around path: ");

        		//Ex: E1F3, min=E, max=F, min=1, max=3
                // Get min/max boundaries between from and to (exclusive)
                int min_file = (from_file < to_file) ? from_file : to_file;
                int max_file = (from_file > to_file) ? from_file : to_file;
                int min_rank = (from_rank < to_rank) ? from_rank : to_rank;
                int max_rank = (from_rank > to_rank) ? from_rank : to_rank;

                for (int f = min_file; f <= max_file; f++) {
                    for (int r = min_rank; r <= max_rank; r++) {
                        // skip start and end
                        if ((f == from_file && r == from_rank) || (f == to_file && r == to_rank))
                            continue;

                        char square[5];
                        square[0] = 'a' + f - 1;
                        square[1] = '0' + r;
                        square[2] = ' ';
                        square[3] = '\0';
                        uart_puts(square);
                    }
                }

                uart_puts("\r\n");
    }
}

void scan_board(void) {
    for (uint8_t mux = 0; mux < 4; mux++) {
        enable_mux(mux);

        for (uint8_t ch = 0; ch < 16; ch++) {
            set_mux_channel(ch);
            HAL_Delay(5); // allow signals to settle

            uint32_t adc_val = read_adc();
            curr_board[mux][ch] = adc_val;
            if (( min_board[mux][ch] != 9999) && (adc_val > max_board[mux][ch])  ){
            	max_board[mux][ch] = adc_val;
            	threshold[mux][ch] = (max_board[mux][ch]+min_board[mux][ch])/2;
            }

            else if(( max_board[mux][ch] != 0) && (adc_val < min_board[mux][ch]) ){
            	min_board[mux][ch] = adc_val;
            	threshold[mux][ch] = (max_board[mux][ch]+min_board[mux][ch])/2;
            }
        }
    }
}

void detect_capture(void) {
	count_piece = 0;
	uint8_t captured_piece = 0;
    for (uint8_t mux = 0; mux < 4; mux++) {
        for (uint8_t ch = 0; ch < 16; ch++) {

            uint8_t prev_state = (prev_board[mux][ch] < prev_threshold[mux][ch]) ? 1 : 0;
            uint8_t curr_state = (curr_board[mux][ch] < threshold[mux][ch]) ? 1 : 0;

            if (curr_state == 1){
            	count_piece += 1;
            }
            if (prev_state && !curr_state) {
            	if(captured_piece == 1)
            	{
            		captured_square[0] = '\0';
					uart_puts("\r\nReturn the piece and try again.\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)("e\n"), strlen("e\n"), UART_TIMEOUT);
					return;
            	}
                strcpy(captured_square, square_map[mux][ch]);
                char msg[32];
                sprintf(msg, "Captured:%s\r\n", captured_square);
                captured_piece+=1;
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        }
    }
    if (captured_piece == 0 || count_piece != total_piece - 1)
    {
    	captured_square[0] = '\0';
    	HAL_UART_Transmit(&huart2, (uint8_t*)("e\n"), strlen("e\n"), UART_TIMEOUT);
		uart_puts("\r\nMake sure piece is in the center.\r\n");
		return;
    }

    if (captured_piece == 1 && count_piece == total_piece - 1)
    {
    	total_piece -= 1;
		memcpy((void*)prev_board, (void*)curr_board, sizeof(prev_board));
		memcpy((void*)prev_threshold, (void*)threshold, sizeof(prev_threshold));
    }
}

char* detect_move(void) {
    static char msg[20];   // ✅ now it's not destroyed when function exits
    msg[0] = '\0';
    count_piece = 0;
    const char *from_squares[2] = {NULL, NULL};
    const char *to_squares[2] = {NULL, NULL};
    uint8_t from_count = 0, to_count = 0;

    for (uint8_t mux = 0; mux < 4; mux++) {
        for (uint8_t ch = 0; ch < 16; ch++) {
            uint8_t prev_state = (prev_board[mux][ch] < prev_threshold[mux][ch]) ? 1 : 0;
            uint8_t curr_state = (curr_board[mux][ch] < threshold[mux][ch]) ? 1 : 0;

            if(curr_state == 1)
            {
            	count_piece += 1;
            }
            if (prev_state && !curr_state && from_count < 2)
                from_squares[from_count++] = square_map[mux][ch];

            if (!prev_state && curr_state && to_count < 2)
                to_squares[to_count++] = square_map[mux][ch];
        }
    }
    if(count_piece != total_piece){
    	return "0";
    }
    // normal move
    if (from_count == 1 && to_count == 1) {
        sprintf(msg, "%s%s", from_squares[0], to_squares[0]);
        uart_puts("\rCase 1\r\n");
    }
    // castling
    else if (from_count == 2 && to_count == 2) {
        const char *from = from_squares[0];
        const char *to   = to_squares[0];

        if (from_squares[1] && from_squares[1][0] == 'e') {
            from = from_squares[1];
            to   = to_squares[0];
        } else if (from_squares[0] && from_squares[0][0] == 'e') {
            from = from_squares[0];
            to   = to_squares[1];
        }

        sprintf(msg, "%s%s", from, to);
        uart_puts("\rCase 2\r\n");
    }

    return msg;
}

void restore_prev_board(void) {
    if (captured_square[0] == '\0') return; // no capture to restore

    for (uint8_t mux = 0; mux < 4; mux++) {
        for (uint8_t ch = 0; ch < 16; ch++) {
            if (strcmp(square_map[mux][ch], captured_square) == 0) {
                prev_board[mux][ch] = prev_threshold[mux][ch] - 1; // restore to "occupied"
                char msg[40];
                sprintf(msg, "Restored:%s\r\n", captured_square);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                break;
            }
        }
    }
    total_piece += 1;
    captured_square[0] = '\0';
}

void update_board_from_reply(const char *reply) {
    if (reply == NULL || strlen(reply) < 4) return;

    char from_square[3], to_square[3];
    strncpy(from_square, reply, 2);
    from_square[2] = '\0';
    strncpy(to_square, reply + 2, 2);
    to_square[2] = '\0';

    // Loop through all squares to find matches
    for (uint8_t mux = 0; mux < 4; mux++) {
        for (uint8_t ch = 0; ch < 16; ch++) {
            if (strcmp(square_map[mux][ch], from_square) == 0) {
                curr_board[mux][ch] = board_empty[mux][ch];
                threshold[mux][ch] = 0;
            }
            else if (strcmp(square_map[mux][ch], to_square) == 0) {
                curr_board[mux][ch] = threshold[mux][ch] - 1;
            }
        }
    }
}

void startup_scan()
{
	scan_board();
	for(int i = 0; i < 4; i++)
	  {
		  for(int j = 0; j < 16; j++)
		  {
			  if(i==0 || i==3)
			  {
				  board_empty[i][j] = ((curr_board[i][j])*100)/80;
				  min_board[i][j] =board_empty[i][j];
				  max_board[i][j] = 0 ;
			  }
			  else
			  {
				  board_empty[i][j] = curr_board[i][j];
				  max_board[i][j] =board_empty[i][j];
				  min_board[i][j] = 9999;
			  }

		  }
	  }
	  for(int i=0; i<4; i++)
		  {
			for(int j=0; j<16; j++)
			{



				if(i==0 ||i ==3)
				{
  //					threshold[i][j] = (board_empty[i][j]*90)/100;
					threshold[i][j] = curr_board[i][j] + 200;

				}
				else
				{
  //        			threshold[i][j] = (board_empty[i][j]*85)/100;
					threshold[i][j] = 0;
				}

			}
		  }

		memcpy((void*)prev_board, (void*)curr_board, sizeof(prev_board));
		memcpy((void*)prev_threshold, (void*)threshold, sizeof(prev_threshold));
		print_board();
		home_reset_sequence();
		HAL_Delay(500);
//		uint8_t xi;
//		uint8_t yi;
//		for(int j=7; j>=0; j--)
//		{
//
//			xi = 1;
//			yi = j;
//			if (corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ) == 0) {
//				HAL_Delay(100);
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//				HAL_Delay(100);
//			}
//
//			xi+=2;
//			if( corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ) == 0) {
//				HAL_Delay(100);
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//				HAL_Delay(100);
//			}
//
//			xi = 0;
//			if (corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ) == 0) {
//				HAL_Delay(100);
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//				HAL_Delay(100);
//			}
//			xi+=2;
//			if (corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ) == 0) {
//				HAL_Delay(100);
//				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//				HAL_Delay(100);
//			}
//
//			 corexy_move_to_cell((uint8_t)1,(uint8_t)yi, DEFAULT_FREQ);
//		}
//		home_reset_sequence();
//		HAL_Delay(500);
//		for(int j=7; j>=0; j--)
//		{
//			xi = 6;
//			yi = j;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi-=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			HAL_Delay(100);
//
//			xi = 7;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi-=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			HAL_Delay(100);
//
//			corexy_move_to_cell((uint8_t)6,(uint8_t)yi, DEFAULT_FREQ);
//		}
//		home_reset_sequence();
//		HAL_Delay(500);
//		scan_board();
//		for(int j=7; j>=0; j--)
//		{
//			xi = 2;
//			yi = j;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi-=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			HAL_Delay(100);
//
//			xi = 3;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi-=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			HAL_Delay(100);
//
//			corexy_move_to_cell((uint8_t)2,(uint8_t)yi, DEFAULT_FREQ);
//		}
//		home_reset_sequence();
//		HAL_Delay(500);
//		for(int j=7; j>=0; j--)
//		{
//			xi = 5;
//			yi = j;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi+=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			HAL_Delay(100);
//
//			xi = 4;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
//			HAL_Delay(100);
//			xi+=2;
//			corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//			corexy_move_to_cell((uint8_t)5,(uint8_t)yi, DEFAULT_FREQ);
//			HAL_Delay(100);
//		}
//		scan_board();
//		home_reset_sequence();
//		HAL_Delay(500);
}
// ส่งข้อความผ่าน UART2
static void uart_puts2(const char s) {
  HAL_UART_Transmit(&huart2, (uint8_t)s, strlen(s), UART_TIMEOUT);
  const uint8_t crlf[2] = {'\r','\n'};
  HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, UART_TIMEOUT);
}


// อ่านบรรทัดผ่าน UART2 (จนเจอ '\r' หรือ '\n')
static int uart2_getline(char *buf, int maxlen) {
  int idx = 0;
  while (idx < maxlen - 1) {
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, UART_TIMEOUT) != HAL_OK) {
      continue; // or return -1 หากอยากให้หลุด
    }


    HAL_UART_Transmit(&huart3, &ch, 1, UART_TIMEOUT);

    if (ch == '\r') {
      // รอ LF ต่อ (ถ้ามี)
      uint8_t next;
      if (HAL_UART_Receive(&huart2, &next, 1, 5) == HAL_OK && next != '\n') {
        // ไม่ใช่ LF ก็เอาคืน buffer (ถ้าต้องการ)
      }
      break;
    } else if (ch == '\n') {
      break;
    } else if (ch == 0x08 || ch == 0x7F) {
      // Backspace
      if (idx > 0) {
        idx--;
      }
    } else {
      buf[idx++] = (char)ch;
    }
  }
  buf[idx] = '\0';
  return idx;
}

void game_over_screen(uint8_t winner)
{
    ILI9341_Fill_Screen(BLACK);

    char buffer[32];

    // ขึ้นข้อความ Game Over
    ILI9341_Draw_Text("GAME OVER", 50, 50, YELLOW, 4, BLACK);

    if (winner == 1)
        sprintf(buffer, " P1 : WIN");
    else if (winner == 2)
        sprintf(buffer, " P2 : WIN");
    else
        sprintf(buffer, "DRAW");

    ILI9341_Draw_Text(buffer, 70, 110, GREEN, 3, BLACK);

    // แสดงข้อความกดเพื่อรีสตาร์ท (optional)
    ILI9341_Draw_Text("Touch to Restart", 60, 170, WHITE, 2, BLACK);
    char ch = 'O';
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

    while (1)
    {
        // รอผู้เล่นแตะจอเพื่อเริ่มใหม่
        if (TP_Touchpad_Pressed())
        {
        	NVIC_SystemReset();
            select_initial_string();
            // reset เวลาใหม่
            player1_time = 600;
            player2_time = 600;
            current_player = 1;
            ILI9341_Fill_Screen(BLACK);
            send_ready = 0;

            update_display();
            HAL_Delay(500); // debounce
            break;  // ออกจาก loop กลับไป main
        }
    }
}

void update_display(void)
{

    char buffer[32];

    // Player 1
    sprintf(buffer, "P1: %02lu:%02lu", player1_time/60, player1_time%60);
    ILI9341_Draw_Text(buffer, 10, 0, WHITE, 3, BLACK);

    // Player 2
    sprintf(buffer, "P2: %02lu:%02lu", player2_time/60, player2_time%60);
    ILI9341_Draw_Text(buffer, 10, 50, WHITE, 3, BLACK);

    // Turn
    sprintf(buffer, "Turn: P%d", current_player);
    ILI9341_Draw_Text(buffer, 10, 100, YELLOW, 3, BLACK);

    // UART Zone

    ILI9341_Draw_Text("Move:", 10, 140, CYAN, 2, BLACK);

    rx_idx = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
}

void ready(char *mode)
{
    ILI9341_Fill_Screen(BLACK);
    ILI9341_Draw_Text("Press button twice to start...", 10, 20, WHITE, 2, BLACK);

    uint32_t last_tick = HAL_GetTick();
    int press_count = 0;
    uint8_t button15_flag = 0; // เก็บสถานะปุ่ม

    if (strcmp(mode, "C") == 0)
    {
        while (1)
        {
            GPIO_PinState button15_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

            // ถ้ากดปุ่มและยังไม่ถูกนับ
            if (button15_state == GPIO_PIN_SET && button15_flag == 0)
            {
            	char test[] = "\r";
                press_count++; // นับครั้งที่กด
                scan_board();
                print_board();
                button15_flag = 1; // ตั้ง flag ว่ากดแล้ว
            }
            // รีเซ็ต flag เมื่อปล่อยปุ่ม
            else if (button15_state == GPIO_PIN_RESET)
            {
                button15_flag = 0;
            }

            if (press_count >= 2) // กดครบ 2 ครั้ง
            {
                ILI9341_Fill_Screen(RED);
                ILI9341_Draw_Text("Start!", 100, 100, WHITE, 3, RED);
                HAL_Delay(200);
                return;
            }

            HAL_Delay(10); // เล็กน้อยเพื่อไม่ให้ loop เร็วจนเกินไป
        }
    }
}

char select_initial_string(void)
{
    char *modes[] = {"C", "P"};   // โหมดทั้งหมด
    int mode_count = 2;
    int current_mode = 0;
    char send_string[16];
    send_ready = 0;



    ILI9341_Fill_Screen(BLACK);
    ILI9341_Draw_Text("Btn1/Btn2 : SelectMode", 10, 20, WHITE, 2, BLACK);
    ILI9341_Draw_Text("Btn3 : ScanBoard", 10, 50, WHITE, 2, BLACK);
    ILI9341_Draw_Text(modes[current_mode], 50, 100, YELLOW, 5, BLACK);

    while(1)
    {
    	button1_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
    	if (last_button_state1 == 0 && button1_state == 1) // กดลงจริง
    	{
    	    HAL_Delay(100); // debounce เล็กน้อย
    	    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0) // ตรวจซ้ำว่ายังกดอยู่
    	    {
    	        current_mode++;
    	        if (current_mode >= mode_count)
    	            current_mode = 0;


    	        ILI9341_Draw_Rectangle(40, 90, 80, 60, BLACK);
    	        ILI9341_Draw_Text(modes[current_mode], 50, 100, YELLOW, 5, BLACK);
    	    }
    	}
    	last_button_state1 = button1_state;

        // ปุ่ม 2 → โหมดถัดไป
    	button2_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    	if (last_button_state2 == 0 && button2_state == 1)
    	{
    	    HAL_Delay(20);  // หน่วงเวลา 20ms เพื่อลด bouncing
    	    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)  // ตรวจซ้ำเพื่อยืนยันว่ากดจริง
    	    {
    	    	HAL_UART_Transmit(&huart2, (uint8_t*)"Z", strlen("Z"), HAL_MAX_DELAY);
    	    }
    	}
    	last_button_state2 = button2_state;

        // ปุ่ม 3 → ส่ง string ผ่าน UART
        button15_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
        if (last_button_state15 == 0 && button15_state == 1)
        {
			char test[] = "\r";


			if (strcmp(modes[current_mode], "C") == 0){
				return 'C';
						}
			//s
			if (strcmp(modes[current_mode], "P") == 0){
				return 'P';
						}



			//
//
//			HAL_UART_Transmit(&huart2, (uint8_t*)send_string, 1, HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart2, (uint8_t*)test, strlen(test), HAL_MAX_DELAY);
//
//			HAL_UART_Transmit(&huart3, (uint8_t*)send_string, 1, HAL_MAX_DELAY);
//			HAL_UART_Transmit(&huart3, (uint8_t*)test, strlen(test), HAL_MAX_DELAY);

        }
        last_button_state15 = button15_state;
    }
}

void process_move(char *rx)
{
    char rx_copy[RX_BUFFER_SIZE];
    strncpy(rx_copy, rx, sizeof(rx_copy));

    char *sep = NULL;
    char *front = NULL;
    char *back  = NULL;
    uint8_t moved = 1;
    uint8_t isCastle = 0;
    uint8_t istake = 0;
    int cut_len = 0;

    if ((sep = strstr(rx_copy, "-t:")) != NULL) {
        sep += 3;
        istake = 1;
        cut_len = 3;
    }
    else if ((sep = strstr(rx_copy, "-n:")) != NULL) {
        sep += 3;
        cut_len = 3;
    }
    else if ((sep = strstr(rx_copy, "-ck:")) != NULL) {
        sep += 4;
        isCastle = 1;
        cut_len = 4;
    }
    else if ((sep = strstr(rx_copy, "-cq:")) != NULL) {
        sep += 4;
        isCastle = 1;
        cut_len = 4;
    }
    else if ((sep = strstr(rx_copy, ":")) != NULL) {
        sep += 1;
        cut_len = 1;
    }

    if (sep) {
        *(sep - cut_len) = '\0';
        front = rx_copy;
        back = sep;
    } else {
        front = rx_copy;
        back = NULL;
    }

    // ทำเหมือนโค้ด main loop เดิมของคุณ
    if (front)
    {
        uint8_t valid_move = 1;
        if(strcmp(front, "t") == 0){
            current_player = (current_player == 1) ? 2 : 1;
            moved = 0;
            valid_move = 0;
        }

        char formatted1[64];
        snprintf(formatted1, sizeof(formatted1), "%.2s to %.2s", front, front+2);

        ILI9341_Draw_Rectangle(0, 140, 240, 180, BLACK);
        if (moved){
            ILI9341_Draw_Text(formatted1, 50, 165, YELLOW, 3, BLACK);
        }

        if(back && moved)
        {
            if (strcmp(back, "nv") == 0)
            {
                ILI9341_Draw_Text("Not-Valid-Move", 40, 190, GREEN, 2, BLACK);
                valid_move = 0;
            }
            else if (strcmp(back, "CHECKMATE") == 0){
                ILI9341_Draw_Text(back, 40, 190, RED, 3, BLACK);
                HAL_Delay(100);

                if (current_player == 1){
                    game_over_screen(1);
                }
                else if (current_player == 2){
                    game_over_screen(2);
                }
            }
            else
            {
                if (isCastle) {
                    ILI9341_Draw_Text(back, 20, 190, GREEN, 3, BLACK);
                    ILI9341_Draw_Text("CASTLE", 130, 190, CYAN, 3, BLACK);
                }
                if (istake) {
                    ILI9341_Draw_Text(back, 10, 190, GREEN, 3, BLACK);
                    ILI9341_Draw_Text("TAKE", 150, 190, RED, 3, BLACK);
                }
                else if (!istake && !isCastle){
                    ILI9341_Draw_Text(back, 60, 190, GREEN, 3, BLACK);
                }
            }
        }
        else if(moved)
        {
            ILI9341_Draw_Text("Invalid", 70, 190, GREEN, 3, BLACK);
            valid_move = 0;
        }

        if(valid_move)
        {
            current_player = (current_player == 1) ? 2 : 1;
            update_display();
        }

        // ส่ง debug
        char msg[128];
        char test[] = "\r";

    }
}



/* ---------- ฟังก์ชัน init USART3 (CubeMX จะ generate ให้) ---------- */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_SPI5_Init();
  MX_RNG_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Stepper_StopAll();
  ILI9341_Init();
  ILI9341_Set_Rotation(1);
  ILI9341_Fill_Screen(BLACK);


  char hello[] = "STM32 UART ready! Type something and press Enter:\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)hello, strlen(hello), HAL_MAX_DELAY);
  char line[LINE_BUFSIZE];
  char state = 'S';
  ILI9341_Fill_Screen(BLACK);
  update_display();               // first draw   // start timer if you use it
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  uint32_t last_tick = HAL_GetTick();

//  home_reset_sequence();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  update_display();
	  if(state=='S'){
		  uart_puts("\r\nSelect Mode> ");
		  char sel = select_initial_string();

		  uart_puts("\r\n Received>");
		  uart_puts(line);

		  if(sel == 'S'){state = 'S';}
		  if(sel == 'P'){state = 'P';}
		  if(sel == 'C'){state = 'C';}

	  }
	  if(state == 'C'){
		  	if(started == 0)
		  	{
		  		startup_scan();
		  		started = 1;
		  		ready("C");
		  		ILI9341_Fill_Screen(BLACK);
		  		update_display();
		  		HAL_TIM_Base_Start_IT(&htim1);
		  	}

		  	button1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
			button2_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
			button15_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
			  if (button2_state == 1 && button2_flag == 0)
			  {
				  strcpy(line, "a");
				  uart_puts(line);
				  button2_flag = 1;
			  }
			  else if (button15_state == 1 && button15_flag == 0)
			  {
				  strcpy(line, "e");
				  uart_puts(line);
				  button15_flag = 1;
			  }
			  else if(button1_state == 1 && button1_flag == 0)
			  {
				  scan_board();
				  print_board();
				  button2_flag = 0;
				  button15_flag = 0;
				  button1_flag = 0;
				  home_reset_sequence();
				  continue;
			  }
			  else
			  {
				  button2_flag = 0;
				  button15_flag = 0;
				  button1_flag = 0;
				  continue;
			  }


			// คำสั่งพิเศษ
			if (strcmp(line, "board") == 0) {
//			  uart_puts("\r\nCurrent board:\r\n");
//			  serialBoard();
			  continue;
			}
			if (line[0] == 'r') {
				home_reset_sequence();
				continue;
			}
			if (line[0] == 'm') {
					HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
//					uart_puts("mg");
					continue;
				}

			if (strcmp(line, "help") == 0) {
//			  uart_puts("\r\nCommands:\r\n  e2e4  - make a move\r\n  board - show board\r\n  help  - this help\r\n");
			  continue;
			}


//			if(is_valid_mov4(line))
//			{
//				strncpy(mov, line, 4);
//			}
//			else
//			{
				char *move = NULL;
				if (line[0] == 'a') {  // กด Enter
					  scan_board();
					  print_board();

					  move = detect_move();
//					  uart_puts(move);
					  if(move[0] == '\0')
					  {
						  uart_puts("\rEmpty\r\n");
//						  uart_puts2("e");
						  continue;
					  }
					  if(move[0] == '0')
					  {
//						  uart_puts("\r\Make sure piece is in the center.\r\n");
//						  uart_puts2("e");
						  continue;
					  }

				  }
				else if (line[0] == 'e') {
					  if (captured_square[0] != '\0')
					  {
//						  uart_puts("\r\nYou already captured something.\r\n");
//						  uart_puts2("e");
						  continue;
					  }

					  scan_board();
					  print_board();
					  detect_capture();
					  continue;
				 }

				if (!is_valid_mov4(move)) {
//				  uart_puts("\r\nInvalid format. Use like e2e4\r\n");
//				  uart_puts2("e");
				  continue;
				}

				strncpy(mov, move, 4);

//			}
			mov[4] = '\0';
			// ✅ เพิ่ม \n ตอนส่งไป UART2
			process_move(mov);

			// --- แสดงผลบน UART3 (TeraTerm) ---


			const char* reply = AI_HvsC();

//			uart_puts("Computer: ");
//			uart_puts(reply);
			if(strcmp(reply, ":nv\n") == 0){
//				uart_puts("\r\nno valid send back");
				restore_prev_board();
				continue;
			}
			HAL_Delay(10);
			memcpy((void*)prev_board, (void*)curr_board, sizeof(prev_board));
			memcpy((void*)prev_threshold, (void*)threshold, sizeof(prev_threshold));
			captured_square[0] = '\0';

			if(strstr(reply, "-c") != NULL){
				if(strstr(reply, "-k") != NULL){motion_castle_left();}
				else if(strstr(reply, "-q") != NULL){motion_castle_right();}
			}
			else{
				if(strstr(reply, "-t") != NULL){
					motion_capture_to_row8((uint8_t)char_to_int(reply[3]),(uint8_t)char_to_int(reply[2]));
//					uart_puts("take!!\r\n");
					total_piece -= 1;
				}
				int yi = char_to_int(reply[0]);
				int xi = char_to_int(reply[1]);
//				uart_puts("\r\nbefore move");
				corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);
				HAL_Delay(500);
				yi = char_to_int(reply[2]);
				xi = char_to_int(reply[3]);
				corexy_move_to_cell((uint8_t)xi,(uint8_t)yi, DEFAULT_FREQ);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);
//				uart_puts("\r\after move\r\n");
			}


			update_board_from_reply(reply);
			memcpy((void*)prev_board, (void*)curr_board, sizeof(prev_board));
			memcpy((void*)prev_threshold, (void*)threshold, sizeof(prev_threshold));
			process_move(reply);
			HAL_Delay(10);

	  }
	  if(state == 'P'){
//	    uart_puts("\r\nPYour move> ");

	    int n = uart_getline(line, sizeof(line));
	    if (n <= 0) {
	      continue;
	    }

	    // ทำให้เป็นตัวพิมพ์เล็ก
	    for (int i = 0; i < n; ++i) {
	      if (line[i] >= 'A' && line[i] <= 'Z') line[i] = (char)(line[i] - 'A' + 'a');
	    }

	    // คำสั่งพิเศษ
	    if (strcmp(line, "board") == 0) {
//	      uart_puts("\r\nCurrent board:\r\n");
//	      serialBoard();
	      continue;
	    }
	    if (line[0] == 'r') {
	      home_reset_sequence();
	      continue;
	    }
	    if (line[0] == 'm') {
	      HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
//	      uart_puts("mg");
	      continue;
	    }
	    if (strcmp(line, "help") == 0) {
//	      uart_puts("\r\nCommands:\r\n  e2e4  - make a move\r\n  board - show board\r\n  help  - this help\r\n");
	      continue;
	    }

	    // ---------------- สร้าง mov ----------------
	    if(is_valid_mov4(line)) {
	      strncpy(mov, line, 4);
	    } else {
	      char *move = NULL;

	      if (line[0] == 'a') {  // โหมดสแกนกระดาน
	        scan_board();
	        print_board();
	        move = detect_move();
	        if (!move || move[0] == '0') {
//	          uart_puts("\r\nMake sure piece is in the center.\r\n");
	          continue;
	        }
	      }
	      else if (line[0] == 'e') { // โหมดตรวจจับการกิน
	        if (captured_square[0] != '\0') {
//	          uart_puts("\r\nYou already captured something.\r\n");
	          continue;
	        }
	        scan_board();
	        print_board();
	        detect_capture();
	        if (captured_square[0] == '\0') {
	          // ยังจับไม่ครบ/ไม่นิ่ง
	          continue;
	        }
	        continue; // โหมด e จบเท่านี้
	      } else {
//	        uart_puts("\r\nInvalid format. Use like e2e4\r\n");
	        continue;
	      }

	      // แปลงเป็นตัวเล็กที่ตัว move จริง ๆ
	      for (int i = 0; move[i]; ++i) {
	        if (move[i] >= 'A' && move[i] <= 'Z')
	          move[i] = (char)(move[i] - 'A' + 'a');
	      }

	      if (!is_valid_mov4(move)) {
//	        uart_puts("\r\nInvalid format. Use like e2e4\r\n");
	        continue;
	      }

	      strncpy(mov, move, 4);
	    }

	    mov[4] = '\0';

	    // ---------- ส่งไป UART2: ต้องส่ง mov + "\n" ----------


	    // --- รอรับ response จาก UART2 ---


	    // --- แสดงผลบน UART3 (TeraTerm) ---
	    // ---------- เรียกเอนจิน HvH (จะคืนสตริง + suffix :CHECK/:CHECKMATE/...) ----------
	    const char* reply = AI_HvsH();


//	    uart_puts("Computer: ");
//	    uart_puts(reply);
	    if(strcmp(reply, ":nv\n") == 0){
//	      uart_puts("\r\nno valid send back");
	      continue;
	    }

	    // (ถ้าต้อง sync กระดานจาก reply ให้ parse แล้ว update_board_from_reply(reply) ที่นี่)
	    memcpy((void*)prev_board, (void*)curr_board, sizeof(prev_board));
	    memcpy((void*)prev_threshold, (void*)threshold, sizeof(prev_threshold));
	    captured_square[0] = '\0';

	    // --- แสดงผลบน UART3 (TeraTerm) รอบสุดท้าย ---
	  }



  }
  home_reset_sequence();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
