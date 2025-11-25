/* micro_max_ported.c — Pure C (C99) port from Arduino sketch using STM32 HAL UART */

#include "main.h"          /* สำหรับ HAL_UART_Transmit และ UART handle */
#include "Micro_Max.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/* ====== ใช้ UART3 ตามตัวอย่าง; ปรับเป็น UART ของคุณได้ ====== */
extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart2;
/* ====== พิมพ์ออก UART: แปลง \n -> \r\n ตามที่ผู้ใช้กำหนด ====== */
void uartPrint_raw(const char *msg) {
  while (*msg) {
    if (*msg == '\n') {
      const uint8_t crlf[2] = {'\r','\n'};
      HAL_UART_Transmit(&huart3, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
    } else {
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, 1, HAL_MAX_DELAY);
    }
    msg++;
  }
}

void uartPrint_to(const char *msg) {
  while (*msg) {
    if (*msg == '\n') {
      const uint8_t crlf[2] = {'\r','\n'};
      HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
    } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, 1, HAL_MAX_DELAY);
    }
    msg++;
  }
}

/* helper เล็ก ๆ สำหรับ printf-style */
void uartPrintf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
//  uartPrint_raw(buf);
}

/* ====== CONFIG / MACROS และตัวแปร global (คงชื่อเดิม) ====== */
#define W while
#define M 0x88
#define S 128
#define I 8000
#define MYRAND_MAX 65535     /* 16bit pseudo random generator */

long  N, T;                  /* N=evaluated positions+S, T=recursion limit */
short Q, O, K, R, k = 16;    /* k=moving side */
char *p, c[5];
static int8_t Z=0; /* p=pointer to c, c=user input, computer output, Z=recursion counter */

char L;
static const int8_t    w[] = {0, 2, 2, 7, -1, 8, 12, 23},                    /* relative piece values    */
     o[] = { -16, -15, -17, 0, 1, 16, 0, 1, 16, 15, 17, 0, 14, 18, 31, 33, 0, /* step-vector lists */
             7, -1, 11, 6, 8, 3, 6,                           /* 1st dir. in o[] per piece*/
             6, 3, 5, 7, 4, 5, 3, 6
           };                                /* initial piece setup      */

/* board is left part, center-pts table is right part, and dummy */
static int8_t b[] = {
  22, 19, 21, 23, 20, 21, 19, 22, 28, 21, 16, 13, 12, 13, 16, 21,
  18, 18, 18, 18, 18, 18, 18, 18, 22, 15, 10,  7,  6,  7, 10, 15,
  0,  0,  0,  0,  0,  0,  0,  0, 18, 11,  6,  3,  2,  3,  6, 11,
  0,  0,  0,  0,  0,  0,  0,  0, 16,  9,  4,  1,  0,  1,  4,  9,
  0,  0,  0,  0,  0,  0,  0,  0, 16,  9,  4,  1,  0,  1,  4,  9,
  0,  0,  0,  0,  0,  0,  0,  0, 18, 11,  6,  3,  2,  3,  6, 11,
  9,  9,  9,  9,  9,  9,  9,  9, 22, 15, 10,  7,  6,  7, 10, 15,
  14, 11, 13, 15, 12, 13, 11, 14, 28, 21, 16, 13, 12, 13, 16, 21, 0
};

char bk[16 * 8 + 1];

unsigned int seed = 0;
uint32_t  byteBoard[8];

char sym[17] = {".?pnkbrq?P?NKBRQ"};
int mn = 1;
char lastH[64], lastM[64];
unsigned short ledv = 1;

/* เดิมใช้ Arduino String; ที่นี่ใช้บัฟเฟอร์ C แทน */
static char inputString[16] = {0};
bool stringComplete = false;

/* ในต้นฉบับมีตัวแปร mov[5]; คุณสามารถประกาศไว้ที่ไฟล์อื่นก็ได้ */
extern char mov[5];

int r;

/* ===== RNG เดิม ===== */
unsigned short myrand(void) {
  unsigned short rr = (unsigned short)(seed % MYRAND_MAX);
  return rr = ((rr << 11) + (rr << 7) + rr) >> 1;
}

/* ===== ประกาศฟังก์ชัน ===== */
void gameOver(void);
void bkp(void);
void serialBoard(void);
short D(short q, short l, short e, unsigned char E, unsigned char z, unsigned char n);

/* ===== Recursive minimax (คงโค้ด/ตรรกะเดิมทุกบรรทัด) ===== */
short D(short q, short l, short e, unsigned char E, unsigned char z, unsigned char n) {
  short m, v, i, P, V, s;
  unsigned char t, p, u, x, y, X, Y, H, B, j, d, h, F, G, C;
  signed char r_;
  if (++Z > 30) {                                   /* stack underrun check */
    --Z; return e;
  }
  q--;                                          /* adj. window: delay bonus */
  k ^= 24;                                      /* change sides             */
  d = Y = 0;                                    /* start iter. from scratch */
  X = myrand() & ~M;                            /* start at random field    */
  W(d++ < n || d < 3 ||                         /* iterative deepening loop */
    z & (K == I) && ((N < T) & (d < 98) ||      /* root: deepen upto time   */
                   (K = X, L = Y & ~M, d = 3)))                /* time's up: go do best    */
  { x = B = X;                                   /* start scan at prev. best */
    h = Y & S;                                   /* request try noncastl. 1st*/
    P = d < 3 ? I : D(-l, 1 - l, -e, S, 0, d - 3); /* Search null move         */
    m = (-P < l) | (R > 35) ? (d > 2 ? -I : e) : -P;   /* Prune or stand-pat   */
    ++N;                                         /* node count (for timing)  */
    do {
      u = b[x];                                   /* scan board looking for   */
      if (u & k) {                                /*  own piece (inefficient!)*/
        r_ = p = u & 7;                           /* p = piece type (set r>0) */
        j = o[p + 16];                             /* first step vector f.piece*/
        W(r_ = p > 2 & r_ < 0 ? -r_ : -o[++j])    /* loop over directions o[] */
        { A:                                        /* resume normal after best */
          y = x; F = G = S;                         /* (x,y)=move, (F,G)=castl.R*/
          do {                                      /* y traverses ray, or:     */
            H = y = h ? (Y ^ h) : (y + r_);          /* sneak in prev. best move */
            if (y & M)break;                         /* board edge hit           */
            m = (E - S & b[E] && y - E < 2 & E - y < 2) ? I : m; /* bad castling */
            if (p < 3 & y == E)H ^= 16;              /* shift capt.sqr. H if e.p.*/
            t = b[H]; if (t & k | p < 3 & !(y - x & 7) - !t)break; /* capt. own, bad pawn mode */
            i = 37 * w[t & 7] + (t & 192);           /* value of capt. piece t   */
            m = i < 0 ? I : m;                       /* K capture                */
            if (m >= l & d > 1)goto C;               /* abort on fail high       */
            v = d - 1 ? e : i - p;                   /* MVV/LVA scoring          */
            if (d - !t > 1)                          /* remaining depth          */
            { v = p < 6 ? b[x + 8] - b[y + 8] : 0;    /* center positional pts.   */
              b[G] = b[H] = b[x] = 0; b[y] = u | 32;  /* do move, set non-virgin  */
              if (!(G & M))b[F] = k + 6, v += 50;     /* castling: put R & score  */
              v -= p - 4 | R > 29 ? 0 : 20;           /* penalize mid-game K move */
              if (p < 3)                              /* pawns:                   */
              { v -= 9 * ((x - 2 & M || b[x - 2] - u) + /* structure, undefended    */
                          (x + 2 & M || b[x + 2] - u) - 1
                          + (b[x ^ 16] == k + 36))
                     - (R >> 2);                       /* end-game push bonus      */
                V = y + r_ + 1 & S ? 647 - p : 2 * (u & y + 16 & 32); /* promo/bonus */
                b[y] += V; i += V;                     /* change piece, add score  */
              }
              v += e + i; V = m > q ? m : q;          /* new eval and alpha       */
              C = d - 1 - (d > 5 & p > 2 & !t & !h);
              C = R > 29 | d < 3 | P - I ? C : d;     /* extend 1 ply if in check */
              do
                s = C > 2 | v > V ? -D(-l, -V, -v,     /* recursive eval. reply */
                                       F, 0, C) : v;    /* or fail low if futile */
              W(s > q & ++C < d); v = s;
              if (z && K - I && v + I && x == K & y == L) /* move pending & in root */
              { Q = -e - i; O = F;                     /* exit if legal & found  */
                R += i >> 7; --Z; return l;            /* captured non-P material*/
              }
              b[G] = k + 6; b[F] = b[y] = 0; b[x] = u; b[H] = t; /* undo move */
            }
            if (v > m)                               /* new best, update max,best*/
              m = v, X = x, Y = y | S & F;            /* mark double move with S  */
            if (h) { h = 0; goto A; }                 /* redo after old best      */
            if (x + r_ - y | u & 32 |
                p > 2 & (p - 4 | j - 7 ||
                         b[G = x + 3 ^ r_ >> 1 & 7] - k - 6 ||
                         b[G ^ 1] | b[G ^ 2])) t += p < 5; /* fake capt for nonslide */
            else F = y;                              /* enable e.p.              */
          } W(!t);
        }
      }
    } W((x = x + 9 & ~M) - B);
C:  if (m > I - M | m < M - I) d = 98;           /* mate holds to any depth  */
    m = (m + I) | (P == I) ? m : 0;              /* best loses K: (stale)mate*/
    if (z && d > 2)
    { *c = 'a' + (X & 7); c[1] = '8' - (X >> 4); c[2] = 'a' + (Y & 7); c[3] = '8' - ((Y >> 4) & 7); c[4] = 0;
      /* เดิมมีบัฟเฟอร์ทิ้งไว้; ไม่พิมพ์อะไรตรงนี้เพื่อคงพฤติกรรม */
    }
  }
  k ^= 24;                                      /* change sides back        */
  --Z; return m += m < e;                       /* delayed-loss bonus       */
}

/* ===== Game over: คงพฤติกรรมเดิม ===== */
void gameOver(void) {
  for (;;) { /* hang */ }
}

/* ===== Backup board ===== */
void bkp(void) {
  for (int i = 0; i < 16 * 8 + 1; i++) {
    bk[i] = b[i];
  }
}


/* ===== แสดงกระดานผ่าน UART ===== */
void serialBoard(void) {
  uartPrint_raw("  +-----------------+\n");
  for (int i = 0; i < 8; i++) {
    char line[64];
    uartPrintf(" %d| ", 8 - i);
    for (int j = 0; j < 8; j++) {
      char ch = sym[b[16 * i + j] & 15];
      line[0] = ch; line[1] = ' '; line[2] = '\0';
      uartPrint_raw(line);
    }
    uartPrint_raw("|\n");
  }
  uartPrint_raw("  +-----------------+\n");
  uartPrint_raw("    a b c d e f g h\n");
}

void board_to_matrix_chars(const signed char *b,
                           const char *sym,
                           char out[8][8],
                           bool rank8_on_row0)
{
    for (int i = 0; i < 8; ++i) {          /* i = 0..7 คือแถวในบอร์ดภายใน (rank 8..1) */
        for (int j = 0; j < 8; ++j) {      /* j = 0..7 คือไฟล์ a..h */
            unsigned code = (unsigned)(b[16 * i + j]) & 0x0F;  /* ตัดมาเฉพาะชนิดชิ้นหมาก */
            char ch = sym[code];

            int row = rank8_on_row0 ? i : (7 - i); /* กลับมุมมองตามต้องการ */
            int col = j;                           /* a..h = 0..7 */

            out[row][col] = ch;
        }
    }
}
/* พิมพ์ board_chars เป็น C literal: char board[8][8] = {{'r','n',...}, ...}; */
void printBoardCharsAsCArray(const char board[8][8], const char *name) {
  char buf[1024];
  int pos = 0;

  if (!name) name = "board_chars";

  pos += snprintf(buf + pos, sizeof(buf) - pos, "char %s[8][8] = {\n", name);
  for (int i = 0; i < 8; ++i) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  {");
    for (int j = 0; j < 8; ++j) {
      char c = board[i][j] ? board[i][j] : ' ';
      /* รองรับการแสดง quote/backslash */
      if (c == '\\' || c == '\'') {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "'\\%c'%s", c, (j<7)?", ":"");
      } else if (c == '\0') {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "'\\0'%s", (j<7)?", ":"");
      } else {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "'%c'%s", c, (j<7)?", ":"");
      }
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "}%s\n", (i<7)?",":"");
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "};\n");

  uartPrint_raw(buf);
}

/* ===== โหมด Human vs Computer (คงโครงเดิม, เปลี่ยน I/O เป็น UART) ===== */
/* One turn: Human vs Computer (micro_max style) */
#include <stdlib.h>   // abs
#include <ctype.h>    // isupper, islower

static int in_bounds(int x, int y) {
    return x >= 0 && x < 8 && y >= 0 && y < 8;
}

/* ตรวจว่าทิศทาง (dx,dy) จาก (x,y) ไป (tx,ty) ว่างตลอดทางหรือไม่
   (ไม่นับปลายทาง) — ใช้กับ R/B/Q */
static int clear_path(int x, int y, int tx, int ty, const char board[8][8]) {
    int dx = (tx > x) - (tx < x);  // -1, 0, หรือ +1
    int dy = (ty > y) - (ty < y);  // -1, 0, หรือ +1

    x += dx; y += dy;
    while (x != tx || y != ty) {
        if (!in_bounds(x,y)) return 0;
        if (board[y][x] != '.') return 0;  // มีตัวขวาง
        x += dx; y += dy;
    }
    return 1;
}

/* คืน 1 ถ้าหมาก piece ที่อยู่ (x,y) โจมตีช่องกษัตริย์ (kx,ky) ได้ โดยคำนึงถึงตัวขวาง */
int attacking(int x, int y, char piece, int kx, int ky, const char board[8][8]) {
    if (!in_bounds(x,y) || !in_bounds(kx,ky)) return 0;

    int dx = kx - x;
    int dy = ky - y;
    int ax = dx < 0 ? -dx : dx;
    int ay = dy < 0 ? -dy : dy;

    switch (piece) {
        case 'P':   // เบี้ยขาว โจมตีเฉียงขึ้น
            return ( (kx == x+1 && ky == y+1) || (kx == x-1 && ky == y+1) );

        case 'p':   // เบี้ยดำ โจมตีเฉียงลง
            return ( (kx == x+1 && ky == y-1) || (kx == x-1 && ky == y-1) );

        case 'N':   // ม้า ไม่โดนตัวขวาง
        case 'n':
            return (ax == 1 && ay == 2) || (ax == 2 && ay == 1);

        case 'K':   // คิง 1 ช่องรอบตัว ไม่โดนตัวขวาง
        case 'k':
            return (ax <= 1 && ay <= 1) && (ax || ay);

        case 'R':   // เรือ: แนวนอน/แนวตั้ง + เส้นทางต้องโล่ง
        case 'r':
            if (kx == x || ky == y) {
                return clear_path(x, y, kx, ky, board);
            }
            return 0;

        case 'B':   // โคน: แนวทแยง + เส้นทางต้องโล่ง
        case 'b':
            if (ax == ay) {
                return clear_path(x, y, kx, ky, board);
            }
            return 0;

        case 'Q':   // ควีน = เรือ หรือ โคน
        case 'q':
            if (kx == x || ky == y || ax == ay) {
                return clear_path(x, y, kx, ky, board);
            }
            return 0;
    }
    return 0;
}

int isKingCheck(char board_chars[8][8])
{
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            char piece = board_chars[y][x];
            if (piece == '.') continue;

            char king = isupper(piece) ? 'k' : 'K';
            for (int ky = 0; ky < 8; ky++) {
                for (int kx = 0; kx < 8; kx++) {
                    if (board_chars[ky][kx] == king) {
                        if (attacking(x, y, piece, kx, ky, board_chars)) {
                            if (isupper(piece)) {
                                return 1; // White piece checking Black king
                            } else {
                                return 2; // Black piece checking White king
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

bool isTaken(const char board[8][8], int fromX, int fromY, int toX, int toY)
{
    if (fromX < 0 || fromX >= 8 || fromY < 0 || fromY >= 8 ||
        toX < 0 || toX >= 8 || toY < 0 || toY >= 8) {
        return false;
    }

    char dst = board[toX][toY];
    char test[1000] = "";

    sprintf(test, "%s\r\n", dst);

    if(board[toY][toX] != '.'){
    	uartPrint_raw("take\r\n");
    	return true;
    }

    uartPrint_raw("no-take 2\r\n");
//    uartPrint_raw(test);
    return false;
}

bool isTakenFromMov(const char board[8][8], const char mov[5]) {
    int fromX = mov[0] - 'a';
    int fromY = 8 - (mov[1] - '0');
    int toX   = mov[2] - 'a';
    int toY   = 8 - (mov[3] - '0');

    return isTaken(board, fromX, fromY, toX, toY);
}

int isCastleMoveType(const char mov[5], const char board[8][8])
{
    int fromX = mov[0] - 'a';
    int fromY = 8 - (mov[1] - '0');
    int toX   = mov[2] - 'a';
    int toY   = 8 - (mov[3] - '0');

    char piece = board[fromY][fromX];
    if (piece != 'K' && piece != 'k') {
        return 0; // ไม่ใช่กษัตริย์
    }

    // ต้องอยู่ใน rank เดิม
    if (fromY != toY) return 0;

    int dx = toX - fromX;

    if (dx == 2) {
        return 1; // Kingside castling
    } else if (dx == -2) {
        return 2; // Queenside castling
    }

    return 0;
}

/* ===== Helpers for side checks ===== */
static inline int is_white_piece(char c) { return c >= 'A' && c <= 'Z'; }
static inline int is_black_piece(char c) { return c >= 'a' && c <= 'z'; }

/* หา king ของฝั่ง */
static int find_king(const char board[8][8], int white, int *kx, int *ky) {
    char target = white ? 'K' : 'k';
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            if (board[y][x] == target) { *kx = x; *ky = y; return 1; }
        }
    }
    return 0;
}

/* ฝั่ง white==1 หมายถึงเช็คว่า "กษัตริย์ขาว" ถูกโจมตีหรือไม่ */
static int is_in_check_side(const char board[8][8], int white) {
    int kx, ky;
    if (!find_king(board, white, &kx, &ky)) {
        /* ถ้าไม่เจอคิง ถือว่า “อันตราย” ไว้ก่อน */
        return 1;
    }
    /* ให้ฝั่งตรงข้ามทุกตัวลองโจมตีกษัตริย์ */
    if (white) {
        /* โจมตีโดยหมาก 'ดำ' (ตัวพิมพ์เล็ก) */
        for (int y = 0; y < 8; ++y)
            for (int x = 0; x < 8; ++x) {
                char pc = board[y][x];
                if (is_black_piece(pc) && attacking(x, y, pc, kx, ky, board))
                    return 1;
            }
    } else {
        /* โจมตีโดยหมาก 'ขาว' (ตัวพิมพ์ใหญ่) */
        for (int y = 0; y < 8; ++y)
            for (int x = 0; x < 8; ++x) {
                char pc = board[y][x];
                if (is_white_piece(pc) && attacking(x, y, pc, kx, ky, board))
                    return 1;
            }
    }
    return 0;
}

/* ทดลองขยับ (fx,fy)->(tx,ty) บนกระดานชั่วคราว แล้วเช็คว่าคิงฝั่งตนเองยังปลอดภัยไหม */
static int try_move_and_safe(const char board[8][8], int fx, int fy, int tx, int ty, int white_turn) {
    if (!in_bounds(fx,fy) || !in_bounds(tx,ty)) return 0;

    char src = board[fy][fx];
    char dst = board[ty][tx];

    if (src == '.' || src == '\0') return 0;

    /* หมากปลายทางต้องไม่ใช่เพื่อนร่วมฝั่ง */
    if (white_turn) {
        if (is_white_piece(dst)) return 0;
    } else {
        if (is_black_piece(dst)) return 0;
    }

    /* ทำสำเนากระดาน */
    char tmp[8][8];
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            tmp[y][x] = board[y][x];

    /* เดินจริงบนสำเนา */
    tmp[ty][tx] = src;
    tmp[fy][fx] = '.';

    /* ห้ามปล่อยให้คิงฝั่งเราโดนรุกหลังเดิน */
    if (is_in_check_side(tmp, white_turn)) return 0;

    return 1; /* เดินได้และปลอดภัย */
}

/* สร้างตาเดิน pseudo-legal สำหรับตัวหมากหนึ่งตัว แล้วเรียก try_move_and_safe
   พบสักหนึ่งตาเดินที่ปลอดภัย -> return 1 */
static int piece_has_legal_move(const char board[8][8], int x, int y, int white_turn) {
    char pc = board[y][x];
    if (pc == '.' || pc == '\0') return 0;

    if (white_turn && !is_white_piece(pc)) return 0;
    if (!white_turn && !is_black_piece(pc)) return 0;

    int dirs[8][2];
    int n_dirs = 0;
    int sliding = 0;

    switch (pc) {
        case 'P': { /* เบี้ยขาว: เดินลง y+1, กินเฉียง y+1,x±1 */
            int ny = y + 1;
            /* เดินตรง 1 ช่องถ้าว่าง */
            if (in_bounds(x, ny) && board[ny][x] == '.') {
                if (try_move_and_safe(board, x, y, x, ny, 1)) return 1;
            }
            /* จับกินเฉียง */
            if (in_bounds(x+1, ny) && is_black_piece(board[ny][x+1])) {
                if (try_move_and_safe(board, x, y, x+1, ny, 1)) return 1;
            }
            if (in_bounds(x-1, ny) && is_black_piece(board[ny][x-1])) {
                if (try_move_and_safe(board, x, y, x-1, ny, 1)) return 1;
            }
            return 0;
        }
        case 'p': { /* เบี้ยดำ: เดินขึ้น y-1, กินเฉียง y-1,x±1 */
            int ny = y - 1;
            if (in_bounds(x, ny) && board[ny][x] == '.') {
                if (try_move_and_safe(board, x, y, x, ny, 0)) return 1;
            }
            if (in_bounds(x+1, ny) && is_white_piece(board[ny][x+1])) {
                if (try_move_and_safe(board, x, y, x+1, ny, 0)) return 1;
            }
            if (in_bounds(x-1, ny) && is_white_piece(board[ny][x-1])) {
                if (try_move_and_safe(board, x, y, x-1, ny, 0)) return 1;
            }
            return 0;
        }
        case 'N': case 'n': {
            /* ม้า: 8 ทิศ ไม่สไลด์ */
            static const int kdx[8] = {+1,+2,+2,+1,-1,-2,-2,-1};
            static const int kdy[8] = {+2,+1,-1,-2,-2,-1,+1,+2};
            for (int i = 0; i < 8; ++i) {
                int tx = x + kdx[i], ty = y + kdy[i];
                if (!in_bounds(tx,ty)) continue;
                char dst = board[ty][tx];
                if (white_turn) {
                    if (dst=='.' || is_black_piece(dst)) {
                        if (try_move_and_safe(board, x, y, tx, ty, 1)) return 1;
                    }
                } else {
                    if (dst=='.' || is_white_piece(dst)) {
                        if (try_move_and_safe(board, x, y, tx, ty, 0)) return 1;
                    }
                }
            }
            return 0;
        }
        case 'B': case 'b':
            sliding = 1;
            n_dirs = 4;
            dirs[0][0]=+1; dirs[0][1]=+1;
            dirs[1][0]=+1; dirs[1][1]=-1;
            dirs[2][0]=-1; dirs[2][1]=+1;
            dirs[3][0]=-1; dirs[3][1]=-1;
            break;
        case 'R': case 'r':
            sliding = 1;
            n_dirs = 4;
            dirs[0][0]=+1; dirs[0][1]=0;
            dirs[1][0]=-1; dirs[1][1]=0;
            dirs[2][0]=0;  dirs[2][1]=+1;
            dirs[3][0]=0;  dirs[3][1]=-1;
            break;
        case 'Q': case 'q':
            sliding = 1;
            n_dirs = 8;
            dirs[0][0]=+1; dirs[0][1]=0;
            dirs[1][0]=-1; dirs[1][1]=0;
            dirs[2][0]=0;  dirs[2][1]=+1;
            dirs[3][0]=0;  dirs[3][1]=-1;
            dirs[4][0]=+1; dirs[4][1]=+1;
            dirs[5][0]=+1; dirs[5][1]=-1;
            dirs[6][0]=-1; dirs[6][1]=+1;
            dirs[7][0]=-1; dirs[7][1]=-1;
            break;
        case 'K': case 'k': {
            /* คิง: 8 ช่องรอบตัว (ไม่ทำ castling ในที่นี้) */
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx==0 && dy==0) continue;
                    int tx = x + dx, ty = y + dy;
                    if (!in_bounds(tx,ty)) continue;
                    char dst = board[ty][tx];
                    if (white_turn) {
                        if (dst=='.' || is_black_piece(dst)) {
                            if (try_move_and_safe(board, x, y, tx, ty, 1)) return 1;
                        }
                    } else {
                        if (dst=='.' || is_white_piece(dst)) {
                            if (try_move_and_safe(board, x, y, tx, ty, 0)) return 1;
                        }
                    }
                }
            return 0;
        }
        default:
            return 0;
    }

    /* สำหรับตัวที่ “สไลด์” (B/R/Q) */
    if (sliding) {
        for (int i = 0; i < n_dirs; ++i) {
            int dx = dirs[i][0], dy = dirs[i][1];
            int tx = x + dx, ty = y + dy;
            while (in_bounds(tx,ty)) {
                char dst = board[ty][tx];
                if (white_turn) {
                    if (dst == '.') {
                        if (try_move_and_safe(board, x, y, tx, ty, 1)) return 1;
                    } else {
                        if (is_black_piece(dst)) {
                            if (try_move_and_safe(board, x, y, tx, ty, 1)) return 1;
                        }
                        break; /* เจอตัวขวางแล้วหยุดทิศนี้ */
                    }
                } else {
                    if (dst == '.') {
                        if (try_move_and_safe(board, x, y, tx, ty, 0)) return 1;
                    } else {
                        if (is_white_piece(dst)) {
                            if (try_move_and_safe(board, x, y, tx, ty, 0)) return 1;
                        }
                        break;
                    }
                }
                tx += dx; ty += dy;
            }
        }
    }

    return 0;
}

/* ===== ฟังก์ชันหลักที่คุณต้องการ ===== */
int canWhiteMove(const char board[8][8]) {
    /* ถ้าคิงขาวหาย ให้ถือว่าเดินไม่ได้ (และจะถูกจับแพ้โดยตรรกะภายนอก) */
    int kx, ky;
    if (!find_king(board, 1, &kx, &ky)) return 0;

    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            if (is_white_piece(board[y][x]))
                if (piece_has_legal_move(board, x, y, 1))
                    return 1;
    return 0;
}

int canBlackMove(const char board[8][8]) {
    int kx, ky;
    if (!find_king(board, 0, &kx, &ky)) return 0;

    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            if (is_black_piece(board[y][x]))
                if (piece_has_legal_move(board, x, y, 0))
                    return 1;
    return 0;
}



const char* AI_HvsC(void) {

  /* เอา 4 ตัวแรกจาก mov มาต่อใน inputString (เลียนแบบ Arduino String +=) */
  size_t cur = strlen(inputString);
  if (cur < sizeof(inputString) - 5) {
    strncat(inputString, mov, 4);
    inputString[sizeof(inputString)-1] = '\0';
  }

  uartPrintf("%d. ", mn);
  {
    char show[8] = {0};
    strncpy(show, inputString, 4);
    show[4] = '\0';
    uartPrint_raw(show);
  }

  /* copy 4 ตัวแรกไปใส่ c[] */
  strncpy(c, inputString, 4);
  c[4] = '\0';

  /* clear input buffer */
  inputString[0] = '\0';
  stringComplete = false;

  uartPrint_raw(" Think ");

  /* -------- [NEW VALIDATION] ตรวจความถูกต้องขั้นต้นเหมือน AI_HvsH() -------- */
  // [FIX] ตรวจพิกัดบนกระดาน
  int fromX = c[0] - 'a';
  int fromY = 8 - (c[1] - '0');
  int toX   = c[2] - 'a';
  int toY   = 8 - (c[3] - '0');

  if (fromX < 0 || fromX >= 8 || fromY < 0 || fromY >= 8 ||
      toX   < 0 || toX   >= 8 || toY   < 0 || toY   >= 8) {
    uartPrint_raw("Invalid move: out of board\n");
    return ":nv\n";
  }

  // [FIX] ตรวจว่ามีตัวหมากที่ต้นทาง
  char srcPieceChar = sym[b[16*fromY + fromX] & 15];
  if (srcPieceChar == '.' || srcPieceChar == '\0') {
    uartPrint_raw("Invalid move: no piece at source\n");
    return ":nv\n";
  }

  // [FIX] ตรวจว่าเดินถูกฝั่ง
  bool whiteTurn = (k == 16);
  bool pieceIsWhite = (srcPieceChar >= 'A' && srcPieceChar <= 'Z');
  if ((whiteTurn && !pieceIsWhite) || (!whiteTurn && pieceIsWhite)) {
    uartPrint_raw("Invalid: wrong side moved\n");
    return ":nv\n";
  }
  /* ------------------------------------------------------------------------ */

  /* ===== ฝั่งมนุษย์ เดิน ===== */
  K = *c - 16 * c[1] + 799;
  L = c[2] - 16 * c[3] + 799; /* parse entered move */

  N = 0;
  T = 0x01;                                 /* T=Computer Play strength */
  bkp();                                    /* Save the board just in case */

  // [FIX] ยืนยันว่าเอนจิน “ยอมรับ” ตาเดินนี้จริง (เหมือนใน AI_HvsH)
  int old_k = k;
  short r_test = D(-I, I, Q, O, 1, 3);      /* ให้เอนจินลอง apply ตาเดินมนุษย์ */

  if (!(r_test > -I + 1) || k == old_k) {
    // [FIX] ตาเดินไม่ถูกต้อง → restore บอร์ด + แจ้ง :nv
    for (int i = 0; i < 16 * 8 + 1; i++) b[i] = bk[i];
    uartPrint_raw("No valid move\n");
    return ":nv\n";
  }

  strcpy(lastH, c);                         /* Valid human movement */

  /* NEW: check after human move */
  {
    char board_chars[8][8];
    board_to_matrix_chars((const signed char*)b, sym, board_chars, true);
    int chk = isKingCheck(board_chars);
    if (chk == 1) {
      uartPrint_raw("HW:CHECK\n");
    } else if (chk == 2) {
      uartPrint_raw("HB:CHECK\n");
    } else {
      uartPrint_raw("HNo-check\n");
    }
  }

  mn++;
  /* Next move */
  char take_snap[8][8];
  board_to_matrix_chars((const signed char*)b, sym, take_snap, true);

  /* ===== ฝั่งคอมพ์ เดิน ===== */
  K = I;
  N = 0;
  T = 0x3F;                                 /* T=Computer Play strength */
  r = D(-I, I, Q, O, 1, 3); /* Think & do */

  if (!(r > -I + 1)) {
    char board_chars[8][8];
    board_to_matrix_chars((const signed char*)b, sym, board_chars, true);
    if(isKingCheck(board_chars) == 1){
//      uartPrint_to("Player CheckMate!\n");
//      uartPrint_raw("Player Win");
//      uartPrint_raw(" CheckMate!\n");
      strcat(lastM, ":CHECKMATE\n");
    }
//    else { uartPrint_to("COM StealthMate!\n"); }
  }

  strcpy(lastM, c);                         /* Valid ARDUINO movement */

  /* [FIX] ติ๊กธง -t/-ck/-cq ให้ถูกต้อง */
  if (isTakenFromMov(take_snap, lastM)) {
    strcat(lastM, "-t");
  }
  {
    int castle = isCastleMoveType(c, take_snap);
    if (castle == 1) { strcat(lastM, "-ck"); }  // kingside
    else if (castle == 2) { strcat(lastM, "-cq"); }  // [FIX] เดิมคุณเช็ค ==1 ซ้ำ
  }

  /* NEW: check immediately after AI move */
  {
    char board_chars[8][8];
    board_to_matrix_chars((const signed char*)b, sym, board_chars, true);
    int chk = isKingCheck(board_chars);

    if (chk == 1) {
//      uartPrint_raw("CHECK (after AI move): White is checking black king.\n");
      strcat(lastM, ":CHECK\n");
    } else if (chk == 2) {
//      uartPrint_raw("CHECK (after AI move): Black is checking white king.\n");
      strcat(lastM, ":CHECK\n");
    } else {
//      uartPrint_raw("No check (after AI move).\n");
      strcat(lastM, ":NO-CHECK\n");
    }
  }
  process_move(lastM);

  r = D(-I, I, Q, O, 1, 3);
  if (!(r > -I + 1)) {
//    uartPrint_raw(lastM);
//    uartPrint_raw("\n");
    strcat(lastM, ":CHECKMATE\n");
  }

  /* แสดงหมาก AI และบอร์ด */
//  uartPrint_raw(lastM);
//  uartPrint_raw("\n");
//  serialBoard();

  /* snapshot บอร์ด (debug) */
  {
    char board_chars[8][8];
    board_to_matrix_chars((const signed char*)b, sym, board_chars, true);
    printBoardCharsAsCArray(board_chars, "snapshot");
  }

  return lastM;
}

const char* AI_HvsH()
{
    // ต่อ mov เข้ากับ inputString
    size_t cur = strlen(inputString);
    if (cur < sizeof(inputString) - 5) {
        strncat(inputString, mov, 4);
        inputString[sizeof(inputString) - 1] = '\0';
    }

//    uartPrintf("%d. ", mn);
    char show[8] = {0};
    strncpy(show, inputString, 4);
    show[4] = '\0';
//    uartPrint_raw(show);

    // เตรียมข้อมูล mov -> c, lastH
    strncpy(c, inputString, 4);
    memset(lastH, 0, sizeof(lastH));
    strncpy(lastH, inputString, 4);
    c[4] = '\0';
    inputString[0] = '\0';
    stringComplete = false;

    // Validate พิกัด
    int fromX = c[0] - 'a';
    int fromY = 8 - (c[1] - '0');
    int toX   = c[2] - 'a';
    int toY   = 8 - (c[3] - '0');

    if (fromX < 0 || fromX >= 8 || fromY < 0 || fromY >= 8 ||
        toX   < 0 || toX   >= 8 || toY   < 0 || toY   >= 8) {
//        uartPrint_raw("Invalid move: out of board\n");
        return ":nv\n";
    }

    // ตรวจว่าต้นทางมีหมากไหม
    char srcPieceChar = sym[b[16*fromY + fromX] & 15];
    if (srcPieceChar == '.' || srcPieceChar == '\0') {
//        uartPrint_raw("Invalid move: no piece at source\n");
        return ":nv\n";
    }

    // ตรวจตาเดินให้ถูกฝั่ง
    bool whiteTurn = (k == 16);
    bool pieceIsWhite = isupper(srcPieceChar);
    if (whiteTurn && !pieceIsWhite) {
//        uartPrint_raw("Invalid: Black moved on White's turn\n");
        return ":nv\n";
    }
    if (!whiteTurn && pieceIsWhite) {
//        uartPrint_raw("Invalid: White moved on Black's turn\n");
        return ":nv\n";
    }

    // สำรองกระดาน
    bkp();

    // ให้ engine ตรวจและลองเดินจริง (ตั้ง K,L แบบเดิม)
    K = *c - 16 * c[1] + 799;
    L = c[2] - 16 * c[3] + 799;
    N = 0;
    T = 0x01;

    int old_k = k;
    short r_test = D(-I, I, Q, O, 1, 3);

    // ถ้า engine ไม่ยอมเดิน → invalid
    if (!(r_test > -I + 1) || k == old_k) {
//        uartPrint_raw("No valid move (engine rejected)\n");
        // restore
        for (int i = 0; i < 16 * 8 + 1; i++) b[i] = bk[i];
        return ":nv\n";
    }

    // ---- เติม flag -ck/-cq/-t จาก snapshot ก่อนหน้า (bk)
    {
        char before[8][8];
        board_to_matrix_chars((const signed char*)bk, sym, before, true);

        bool isTake = isTakenFromMov(before, lastH); // ของเดิมคุณมีอยู่แล้ว

        int fx = lastH[0] - 'a';
        int fy = 8 - (lastH[1] - '0');
        int tx = lastH[2] - 'a';
        int ty = 8 - (lastH[3] - '0');

        char piece = before[fy][fx];
        int dx = tx - fx, dy = ty - fy;

        if ((piece == 'K' || piece == 'k') && dy == 0 && (dx == 2 || dx == -2)) {
            if (dx ==  2) strncat(lastH, "-ck", sizeof(lastH) - strlen(lastH) - 1);
            else          strncat(lastH, "-cq", sizeof(lastH) - strlen(lastH) - 1);
        } else if (isTake) {
            strncat(lastH, "-t", sizeof(lastH) - strlen(lastH) - 1);
        }
    }

    // ---- ตรวจ CHECK / CHECKMATE / STALEMATE หลังเดิน ----
    {
        char board_chars[8][8];
        board_to_matrix_chars((const signed char*)b, sym, board_chars, true);

        // isKingCheck: 1=white in check, 2=black in check, 0=none
        int chk = isKingCheck(board_chars);
        int blackmov = canBlackMove(board_chars);
        int whitemov = canWhiteMove(board_chars);
        int whiteToMove = (k == 16);   // k มาจาก engine เดิมของคุณ: 16 = ถึงตาขาวเดิน

            int inCheck, hasMove;
            if (whiteToMove) {
                inCheck = (chk == 2);      // 2 = black piece checking White king
                hasMove = whitemov;
            } else {
                inCheck = (chk == 1);      // 1 = white piece checking Black king
                hasMove = blackmov;
            }

            if (!hasMove) {
                if (inCheck) {
//                    uartPrint_raw("CHECKMATE\n");
                    // uartPrint_to("CHECKMATE\n");
                    strcat(lastH, ":CHECKMATE\n");
                } else {
//                    uartPrint_raw("STALEMATE\n");
                    // uartPrint_to("STALEMATE\n");
                    strcat(lastH, ":STALEMATE\n");
                }
            } else {
                if (inCheck) {
//                    uartPrint_raw("CHECK\n");
//                    uartPrint_to(":CHECK\n");
                    strcat(lastH, ":CHECK\n");
                } else {
//                    uartPrint_raw("NO-CHECK\n");
//                    uartPrint_to(":NO-CHECK\n");
                	strcat(lastH, ":NO-CHECK\n");
                }
            }

    }

//    uartPrint_raw("\n");
//    uartPrint_raw(lastH);
//    uartPrint_raw("\n");
//    serialBoard();

    mn++;
    return lastH;
}
