/**
 * @file macros.h
 * @brief Header file containing macros and typedefs.
 * @date Jun 3, 2024
 * @author Elyass Jaoudat
 */

#pragma once

#ifndef INC_MACROS_H_
#define INC_MACROS_H_



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

/* Type casting */
#define MAKE_SHORT(hi, lo)      ((uint16_t)(((uint16_t)(hi) << 8) | (lo)))
#define MAKE_DWORD(b1, b2, b3, b4) \
    ((uint32_t)(b1) << 24 | \
     (uint32_t)(b2) << 16 | \
     (uint32_t)(b3) <<  8 | \
     (uint32_t)(b4))

/* Debug */
#ifdef DEBUG
#define DEBUG_PRINT(x)              printf("[NFC Reader] "); printf(x); printf("\r\n")
#define DEBUG_PRINTBUF(buf, len)    for (int i = 0; i < len; i++) printf("%02X ", buf[i])
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTBUF(buf, len)
#endif


/* Defines -------------------------------------------------- */


#endif /* INC_MACROS_H_ */