/**
 * @file iso14443.h
 * @brief Header file containing ISO14443-3A and ISO14443-4 definitions.
 * @date Aug 6, 2024
 * @author Elyass Jaoudat
 * 
 */

#pragma once

#ifndef INC_ISO14443_H_
#define INC_ISO14443_H_

#include "macros.h"

/* Constants --------------------------------------------------------- */

/* Error codes */
#define ISO14443A_SUCCESS               ((byte) 0x00)
#define ISO14443A_ERROR                 ((byte) 0x01)
#define ISO14443A_ERROR_CRC             ((byte) 0x02)
#define ISO14443A_ERROR_PARITY          ((byte) 0x03)
#define ISO14443A_ERROR_COLLISION       ((byte) 0x04)
#define ISO14443A_ERROR_NO_RESPONSE     ((byte) 0x05)


/* ISO14443 Type A */
#define REQA    ((byte) 0x26)
#define WUPA    ((byte) 0x52)
#define SELECT  ((byte) 0x93)
#define RATS    ((byte) 0xE0)

#define UID_MASK            ((byte) 0xC0)
#define UID_SINGLE          ((byte) 0x00)
#define UID_DOUBLE          ((byte) 0x01 << 6)
#define UID_TRIPLE          ((byte) 0x02 << 6)

#define UID_SINGLE_SIZE     ((byte) 0x04)
#define UID_DOUBLE_SIZE     ((byte) 0x07)
#define UID_TRIPLE_SIZE     ((byte) 0x0A)

/* Exported Macros --------------------------------------------------- */
/* Exported Typedefs ------------------------------------------------- */
typedef enum {
    PCD_STANDBY,
    PCD_READY,
    PCD_POLLING,
    PCD_LOOP
} pcd_state;

/* Exported Functions ------------------------------------------------ */




#endif /* INC_ISO14443_H_ */