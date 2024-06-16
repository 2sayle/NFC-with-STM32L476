/**
 * @file macros.h
 * @brief Header file containing macros and typedefs.
 * @date Jun 3, 2024
 * @author Elyass Jaoudat
 */

#ifndef INC_MACROS_H_
#define INC_MACROS_H_

/* Exported typdefs ----------------------------------------- */
typedef uint8_t byte;
typedef uint16_t ushort;
typedef uint32_t word;
typedef uint64_t dword;

/* Exported Macros ------------------------------------------ */

/* Binary Operations */
#define SET_BIT(byte, bit)      ((byte) | (1U << (bit)))
#define RESET_BIT(byte, bit)    ((byte) & ~(1U << (bit)))
#define TOGGLE_BIT(byte, bit)   ((byte) ^ (1U << (bit)))
#define READ_BIT(byte, bit)     (((byte) >> (bit)) & 0x01)

/* Register Operations */
#define SET_REG(reg, value)             ((reg) |= (value))
#define RESET_REG(reg, value)           ((reg) &= ~(value))
#define READ_REG(reg, value)            ((reg) & (value))
#define MODIFY_REG(reg, value, mask)    ((reg) = ((reg) & ~(mask)) | (value))


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


/* Defines -------------------------------------------------- */


#endif /* INC_MACROS_H_ */