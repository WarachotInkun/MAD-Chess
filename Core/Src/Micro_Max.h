#ifndef MICRO_MAX_H
#define MICRO_MAX_H

#include <stdint.h>
#include <stdbool.h>



#ifdef cplusplus
extern "C" {
#endif

extern char mov[5];

const char* AI_HvsC(void);
const char* AI_HvsH(void);
void serialBoard(void);      // <-- เพิ่มบรรทัดนี้ให้ main.c มองเห็น
void bkp(void);              // (ไม่จำเป็นถ้า main ไม่ใช้)
short D(short q, short l, short e, unsigned char E, unsigned char z, unsigned char n);

#ifdef cplusplus
}
#endif

#endif /* MICRO_MAX_H */
