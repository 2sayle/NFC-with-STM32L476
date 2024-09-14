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
#define ISO_SUCCESS               ((byte) 0x00)
#define ISO_ERROR                 ((byte) 0x01)
#define ISO_ERROR_LEN             ((byte) 0x02)
#define ISO_ERROR_CRC             ((byte) 0x03)
#define ISO_ERROR_PARITY          ((byte) 0x04)
#define ISO_ERROR_COLLISION       ((byte) 0x05)
#define ISO_ERROR_NO_RESPONSE     ((byte) 0x06)
#define ISO_ERROR_TIMEOUT         ((byte) 0x07)

/* --------------------------- */
/* ----- ISO14443 Type A ----- */
/* --------------------------- */

/* Anticollision commands */
#define REQA    ((byte) 0x26)
#define WUPA    ((byte) 0x52)
#define SELECT  ((byte) 0x93)
#define RATS    ((byte) 0xE0)

/* Unique IDentifier */
#define UID_MASK            ((byte) 0xC0)
#define UID_SINGLE          ((byte) 0x00)
#define UID_DOUBLE          ((byte) 0x01 << 6)
#define UID_TRIPLE          ((byte) 0x02 << 6)
#define UID_SINGLE_SIZE     ((byte) 0x04)
#define UID_DOUBLE_SIZE     ((byte) 0x07)
#define UID_TRIPLE_SIZE     ((byte) 0x0A)

/* ATS Parameters */
#define ATS_MAX_LEN     64

/* ----- Interface Parameters ----- */

/* Frame Size */
#define FSCI0    0x08
#define FSDI0    0x08
#define FSC0     ((ushort) 1 << FSCI0)     
#define FSD0     ((ushort) 1 << FSDI0)


/* PPS */
#define PPSS_MASK   ((byte) 0xD0)
#define PPS0_MASK   ((byte) 0xEF)
#define PPS1_MASK   ((byte) 0x0F)

/* APDU and Blocks */
#define I_BLOCK_MASK    ((byte) 0xE2)
#define R_BLOCK_MASK    ((byte) 0xE6)
#define S_BLOCK_MASK    ((byte) 0xC5)

#define I_BLOCK_PATTERN ((byte) 0x02)
#define R_BLOCK_PATTERN ((byte) 0xA2)
#define S_BLOCK_PATTERN ((byte) 0xC0)

#define CID_MASK        ((byte) 1 << 3)
#define NAD_MASK        ((byte) 1 << 2)

/* ----- Timings parameters ----- */
#define FC_INV_MS   (44000 / 555978)  // 78.8 us = 1/fc (fc = 13.56 MHz) in ms

#define SFGT_MAX_MS     4949
#define FWT_MAX_MS      4949

#define WTXM_MASK       ((byte) 0x3F)
#define WTX_RESP_MASK   ((byte) 0xC0)

/* ----- S-Parameters ----- */
#define S_PARAM_TAG_INFO        ((byte) 0xA0)

#define S_PARAM_TAG_ERROR       ((byte) 0xBE)
#define S_PARAM_LENGTH_ERROR    ((byte) 0x01
#define S_PARAM_VALUE_ERROR     ((byte) 0x00)

#define S_PARAM_TAG_BITRATE_REQ         ((byte) 0xA1)
#define S_PARAM_TAG_BITRATE_INFO        ((byte) 0xA2)
#define S_PARAM_TAG_BITRATE_ACTIVATE    ((byte) 0xA3)
#define S_PARAM_TAG_BITRATE_ACK         ((byte) 0xA4)

#define S_PARAM_TAG_BITRATES_PCD_TO_PICC_SUPPORTED        ((byte) 0x80)
#define S_PARAM_TAG_BITRATES_PICC_TO_PCD_SUPPORTED        ((byte) 0x81)
#define S_PARAM_TAG_FRAMEOPT_PICC_TO_PCD_SUPPORTED        ((byte) 0x82)

#define S_PARAM_TAG_BITRATES_PCD_TO_PICC_SELECTED        ((byte) 0x83)
#define S_PARAM_TAG_BITRATES_PICC_TO_PCD_SELECTED        ((byte) 0x84)
#define S_PARAM_TAG_FRAMEOPT_PICC_TO_PCD_SELECTED        ((byte) 0x85)


/* Exported Macros --------------------------------------------------- */

/* Exported Typedefs ------------------------------------------------- */
typedef enum {
    PCD_STANDBY,
    PCD_READY,
    PCD_POLLING,
    PCD_LOOP
} pcd_state;

typedef struct {
    byte TL;
    byte T0;
    byte TA1;
    byte TB1;
    byte TC1;
    byte TH[ATS_MAX_LEN];
    byte NbHist;
    byte *atsBuffer;
} ats_t;


/* Exported Functions ------------------------------------------------ */

ushort ISO_ComputeCRC16(byte *, word);
byte ISO_StoreATS(byte *, byte, ats_t *);
byte ISO_PerformRATS(byte *, ushort *);
byte ISO_ExchangePPS(byte *, byte);
byte ISO_SendBlock(byte *, ushort len);
byte ISO_ReceiveBlock(byte *, ushort *len);
byte ISO_ExchangeAPDU(byte *, ushort len);



#endif /* INC_ISO14443_H_ */