/**
 * @file macros.h
 * @brief Header file containing macros and typedefs.
 * @date Jun 3, 2024
 * @author Elyass Jaoudat
 */

#pragma once

#ifndef INC_MACROS_H_
#define INC_MACROS_H_


/* Includes ----------------------------------------------- */
#include <stdint.h>
#include <stdio.h>

/* Exported typdefs ----------------------------------------- */
typedef uint8_t  byte;
typedef uint16_t ushort;
typedef uint32_t word;

/* Exported Macros ------------------------------------------ */


/* Miscellaneous */
#define ever (;;)

/* Binary Operations */
#define ST25R_SET_BIT(byte, bit)      ((byte) = (byte) | (1U << (bit)))
#define ST25R_RESET_BIT(byte, bit)    ((byte) = (byte) & ~(1U << (bit)))
#define ST25R_TOGGLE_BIT(byte, bit)   ((byte) = (byte) ^ (1U << (bit)))
#define ST25R_READ_BIT(byte, bit)     ((byte) = ((byte) >> (bit)) & 0x01)

/* Register Operations */
#define ST25R_SET_REG(reg, value)             ((reg) |= (value))
#define ST25R_RESET_REG(reg, value)           ((reg) &= ~(value))
#define ST25R_READ_REG(reg, value)            ((reg) & (value))
#define ST25R_MODIFY_REG(reg, value, mask)    ((reg) = ((reg) & ~(mask)) | (value))

/* Bit masks */
#define BIT0_MASK(x)    (x & 0x01)
#define BIT1_MASK(x)    (x & 0x02)
#define BIT2_MASK(x)    (x & 0x04)
#define BIT3_MASK(x)    (x & 0x08)
#define BIT4_MASK(x)    (x & 0x10)
#define BIT5_MASK(x)    (x & 0x20)
#define BIT6_MASK(x)    (x & 0x40)
#define BIT7_MASK(x)    (x & 0x80)

/* "Logical" bit masks */
#define LBIT0_MASK(x)    (BIT0_MASK(x) >> 0)
#define LBIT1_MASK(x)    (BIT1_MASK(x) >> 1)
#define LBIT2_MASK(x)    (BIT2_MASK(x) >> 2)
#define LBIT3_MASK(x)    (BIT3_MASK(x) >> 3)
#define LBIT4_MASK(x)    (BIT4_MASK(x) >> 4)
#define LBIT5_MASK(x)    (BIT5_MASK(x) >> 5)
#define LBIT6_MASK(x)    (BIT6_MASK(x) >> 6)
#define LBIT7_MASK(x)    (BIT7_MASK(x) >> 7)


/* Type casting */
#define MAKE_SHORT(hi, lo)      ((uint16_t)(((uint16_t)(hi) << 8) | (lo)))

/* Debug */
#if defined(DEBUG)
    #define DEBUG_PRINT(str)              printf("[NFC05A1 TCL Reader] "); printf(str); printf("\r\n")
    #define DEBUG_PRINTBUF(buf, len)      for (int i = 0; i < len; i++) {printf("%02X ", buf[i])}
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTBUF(buf, len)
#endif


/* Defines -------------------------------------------------- */


#endif /* INC_MACROS_H_ */