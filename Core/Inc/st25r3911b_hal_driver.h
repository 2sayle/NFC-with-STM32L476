/**
 * @file st25r3911b_hal_driver.h
 * @brief This file contains the header file for the ST25R3911B NFC controller HAL driver.
 *
 * This file provides the necessary functions and constants to initialize and control the ST25R3911B NFC controller
 * using the HAL (Hardware Abstraction Layer) library provided by STMicroelectronics.
 *
 * @date Jun 2, 2024
 * @author Elyass Jaoudat
 */

#pragma once

#ifndef INC_ST25R3911B_HAL_DRIVER_H_
#define INC_ST25R3911B_HAL_DRIVER_H_


/* Includes ------------------------------------------------- */
#include "macros.h"

/* Macros --------------------------------------------------- */

/* Constants ------------------------------------------------ */

/* SPI Operating Modes Masks */
#define ST25R_REG_WRITE        ((byte) 0x00)
#define ST25R_REG_READ         ((byte) 0x40)
#define ST25R_FIFO_LOAD        ((byte) 0x80)
#define ST25R_FIFO_READ        ((byte) 0xBF)
#define ST25R_DIRECT_CMD       ((byte) 0xC0)

/* Registers addresses */
#define ST25R_IO_CONF1_REG                 ((byte) 0x00)
#define ST25R_IO_CONF2_REG                 ((byte) 0x01)
#define ST25R_OP_CONTROL_REG               ((byte) 0x02)
#define ST25R_MODE_DEF_REG                 ((byte) 0x03)
#define ST25R_BIT_RATE_REG                 ((byte) 0x04)
#define ST25R_ISO14443A_NFC_REG            ((byte) 0x05)
#define ST25R_ISO14443B_REG                ((byte) 0x06)
#define ST25R_ISO14443B_FELICA_REG         ((byte) 0x07)
#define ST25R_STREAM_MODE_REG              ((byte) 0x08)
#define ST25R_AUX_DEF_REG                  ((byte) 0x09)
#define ST25R_RX_CONF1_REG                 ((byte) 0x0A)
#define ST25R_RX_CONF2_REG                 ((byte) 0x0B)
#define ST25R_RX_CONF3_REG                 ((byte) 0x0C)
#define ST25R_RX_CONF4_REG                 ((byte) 0x0D)
#define ST25R_MASK_RX_TIMER_REG            ((byte) 0x0E)
#define ST25R_NO_RESP_TIMER1_REG           ((byte) 0x0F)
#define ST25R_NO_RESP_TIMER2_REG           ((byte) 0x10)
#define ST25R_GPT_NO_RESP_REG              ((byte) 0x11)
#define ST25R_GPT1_REG                     ((byte) 0x12)
#define ST25R_GPT2_REG                     ((byte) 0x13)
#define ST25R_MASK_MAIN_INTERRUPT_REG      ((byte) 0x14)
#define ST25R_MASK_TIMER_NFC_INT_REG       ((byte) 0x15)
#define ST25R_MASK_ERROR_REG               ((byte) 0x16)
#define ST25R_MAIN_INTERRUPT_REG           ((byte) 0x17)
#define ST25R_TIMER_NFC_INT_REG            ((byte) 0x18)
#define ST25R_ERROR_WUP_INTERRUPT_REG      ((byte) 0x19)
#define ST25R_FIFO1_STATUS_REG             ((byte) 0x1A)
#define ST25R_FIFO2_STATUS_REG             ((byte) 0x1B)
#define ST25R_COLL_DISPLAY_REG             ((byte) 0x1C)
#define ST25R_NB1_TX_BYTES_REG             ((byte) 0x1D)
#define ST25R_NB2_TX_BYTES_REG             ((byte) 0x1E)
#define ST25R_NFCIP_BR_DETECT_REG          ((byte) 0x1F)
#define ST25R_AD_CONV_OUT_REG              ((byte) 0x20)
#define ST25R_ANT_CAL_CONTROL_REG          ((byte) 0x21)
#define ST25R_ANT_CAL_TARGET_REG           ((byte) 0x22)
#define ST25R_ANT_CAL_DISPLAY_REG          ((byte) 0x23)
#define ST25R_AM_MOD_DEPTH_CONTROL_REG     ((byte) 0x24)
#define ST25R_AM_MOD_DEPTH_DISPLAY_REG     ((byte) 0x25)
#define ST25R_RFO_AM_MOD_LVL_DEF_REG       ((byte) 0x26)
#define ST25R_RFO_NORMAL_LVL_DEF_REG       ((byte) 0x27)
#define ST25R_EXT_FIELD_DET_THRESH_REG     ((byte) 0x29)
#define ST25R_REGUL_VOLTAGE_CTRL_REG       ((byte) 0x2A)
#define ST25R_REGUL_TIM_DISPLAY_REG        ((byte) 0x2B)
#define ST25R_RSSI_DISPLAY_REG             ((byte) 0x2C)
#define ST25R_GAIN_REDUCTION_STATE_REG     ((byte) 0x2D)
#define ST25R_CAP_SENSOR_CONTROL_REG       ((byte) 0x2E)
#define ST25R_CAP_SENSOR_DISPLAY_REG       ((byte) 0x2F)
#define ST25R_AUX_DISPLAY_REG              ((byte) 0x30)
#define ST25R_WUP_TIM_CONTROL_REG          ((byte) 0x31)
#define ST25R_AMP_MEAS_CONF_REG            ((byte) 0x32)
#define ST25R_AMP_MEAS_REF_REG             ((byte) 0x33)
#define ST25R_AMP_AUTO_AVG_DISPLAY_REG     ((byte) 0x34)
#define ST25R_AMP_MEAS_DISPLAY_REG         ((byte) 0x35)
#define ST25R_PHASE_MEAS_CONF_REG          ((byte) 0x36)
#define ST25R_PHASE_MEAS_REF_REG           ((byte) 0x37)
#define ST25R_PHASE_AUTO_AVG_DISPLAY_REG   ((byte) 0x38)
#define ST25R_PHASE_MEAS_DISPLAY_REG       ((byte) 0x39)
#define ST25R_CAP_MEAS_CONF_REG            ((byte) 0x3A)
#define ST25R_CAP_MEAS_REF_REG             ((byte) 0x3B)
#define ST25R_CAP_AUTO_AVG_DISPLAY_REG     ((byte) 0x3C)
#define ST25R_CAP_MEAS_DISPLAY_REG         ((byte) 0x3D)
#define ST25R_IC_IDENTITY_REG              ((byte) 0x3F)

/* Direct commands */
#define ST25R_SET_DEFAULT_CMD              ((byte) 0xC1)
#define ST25R_CLEAR_CMD                    ((byte) 0xC2)
#define ST25R_TRANSMIT_WITH_CRC_CMD        ((byte) 0xC4)
#define ST25R_TRANSMIT_WITHOUT_CRC_CMD     ((byte) 0xC5)
#define ST25R_TRANSMIT_REQA_CMD            ((byte) 0xC6)
#define ST25R_TRANSMIT_WUPA_CMD            ((byte) 0xC7)
#define ST25R_NFC_INITIAL_FIELD_ON_CMD     ((byte) 0xC8)
#define ST25R_NFC_RESPONSE_FIELD_ON_CMD    ((byte) 0xC9)
#define ST25R_NFC_RESP_FIELD_N0_CMD        ((byte) 0xCA)
#define ST25R_NFC_NORMAL_MODE_CMD          ((byte) 0xCB)
#define ST25R_ANALOG_PRESET_CMD            ((byte) 0xCC)
#define ST25R_MASK_RX_DATA_CMD             ((byte) 0xD0)
#define ST25R_UNMASK_RX_DATA_CMD           ((byte) 0xD1)
#define ST25R_MEAS_AMP_CMD                 ((byte) 0xD3)
#define ST25R_SQUELCH_CMD                  ((byte) 0xD4)
#define ST25R_RESET_RX_GAIN_CMD            ((byte) 0xD5)
#define ST25R_ADJUST_REGULATORS_CMD        ((byte) 0xD6)
#define ST25R_CAL_MOD_DEPTH_CMD            ((byte) 0xD7)
#define ST25R_CAL_ANTENNA_CMD              ((byte) 0xD8)
#define ST25R_MEAS_PHASE_CMD               ((byte) 0xD9)
#define ST25R_CLEAR_RSSI_CMD               ((byte) 0xDA)
#define ST25R_TRANSPARENT_MODE_CMD         ((byte) 0xDC)
#define ST25R_CAL_CAP_SENSOR_CMD           ((byte) 0xDD)
#define ST25R_MEAS_CAP_CMD                 ((byte) 0xDE)
#define ST25R_MEAS_PWR_SUPPLY_CMD          ((byte) 0xDF)
#define ST25R_START_GPTIM_CMD              ((byte) 0xE0)
#define ST25R_START_WUTIM_CMD              ((byte) 0xE1)
#define ST25R_START_MRTIM_CMD              ((byte) 0xE2)
#define ST25R_START_NRTIM_CMD              ((byte) 0xE3)
#define ST25R_TEST_ACCESS_CMD              ((byte) 0xFC)

/* AM Modulation indices */
#define ST25R_MOD_INDEX_0_0        ((byte) 0x00)
#define ST25R_MOD_INDEX_0_8        ((byte) 0x01)
#define ST25R_MOD_INDEX_1_5        ((byte) 0x02)
#define ST25R_MOD_INDEX_2_3        ((byte) 0x03)
#define ST25R_MOD_INDEX_3_0        ((byte) 0x04)
#define ST25R_MOD_INDEX_3_8        ((byte) 0x05)
#define ST25R_MOD_INDEX_4_5        ((byte) 0x06)
#define ST25R_MOD_INDEX_5_2        ((byte) 0x07)
#define ST25R_MOD_INDEX_5_9        ((byte) 0x08)
#define ST25R_MOD_INDEX_6_6        ((byte) 0x09)
#define ST25R_MOD_INDEX_7_2        ((byte) 0x0A)
#define ST25R_MOD_INDEX_7_9        ((byte) 0x0B)
#define ST25R_MOD_INDEX_8_6        ((byte) 0x0C)
#define ST25R_MOD_INDEX_9_2        ((byte) 0x0D)
#define ST25R_MOD_INDEX_9_9        ((byte) 0x0E)
#define ST25R_MOD_INDEX_10_5       ((byte) 0x0F)
#define ST25R_MOD_INDEX_11_1       ((byte) 0x10)
#define ST25R_MOD_INDEX_11_7       ((byte) 0x11)
#define ST25R_MOD_INDEX_12_3       ((byte) 0x12)
#define ST25R_MOD_INDEX_12_9       ((byte) 0x13)
#define ST25R_MOD_INDEX_13_5       ((byte) 0x14)
#define ST25R_MOD_INDEX_14_1       ((byte) 0x15)
#define ST25R_MOD_INDEX_14_7       ((byte) 0x16)
#define ST25R_MOD_INDEX_15_2       ((byte) 0x17)
#define ST25R_MOD_INDEX_15_8       ((byte) 0x18)
#define ST25R_MOD_INDEX_16_3       ((byte) 0x19)
#define ST25R_MOD_INDEX_16_9       ((byte) 0x1A)
#define ST25R_MOD_INDEX_17_4       ((byte) 0x1B)
#define ST25R_MOD_INDEX_17_9       ((byte) 0x1C)
#define ST25R_MOD_INDEX_18_5       ((byte) 0x1D)
#define ST25R_MOD_INDEX_19_0       ((byte) 0x1E)
#define ST25R_MOD_INDEX_19_5       ((byte) 0x1F)
#define ST25R_MOD_INDEX_20_0       ((byte) 0x64)
#define ST25R_MOD_INDEX_20_5       ((byte) 0x65)
#define ST25R_MOD_INDEX_21_0       ((byte) 0x66)
#define ST25R_MOD_INDEX_21_5       ((byte) 0x67)
#define ST25R_MOD_INDEX_22_0       ((byte) 0x68)
#define ST25R_MOD_INDEX_22_4       ((byte) 0x69)
#define ST25R_MOD_INDEX_22_9       ((byte) 0x6A)
#define ST25R_MOD_INDEX_23_4       ((byte) 0x6B)
#define ST25R_MOD_INDEX_23_8       ((byte) 0x6C)
#define ST25R_MOD_INDEX_24_3       ((byte) 0x6D)
#define ST25R_MOD_INDEX_24_7       ((byte) 0x6E)
#define ST25R_MOD_INDEX_25_1       ((byte) 0x6F)
#define ST25R_MOD_INDEX_25_6       ((byte) 0x70)
#define ST25R_MOD_INDEX_26_0       ((byte) 0x71)
#define ST25R_MOD_INDEX_26_4       ((byte) 0x72)
#define ST25R_MOD_INDEX_26_9       ((byte) 0x73)
#define ST25R_MOD_INDEX_27_3       ((byte) 0x74)
#define ST25R_MOD_INDEX_27_7       ((byte) 0x75)
#define ST25R_MOD_INDEX_28_1       ((byte) 0x76)
#define ST25R_MOD_INDEX_28_5       ((byte) 0x77)
#define ST25R_MOD_INDEX_28_9       ((byte) 0x78)
#define ST25R_MOD_INDEX_29_3       ((byte) 0x79)
#define ST25R_MOD_INDEX_29_7       ((byte) 0x7A)
#define ST25R_MOD_INDEX_30_1       ((byte) 0x7B)
#define ST25R_MOD_INDEX_30_4       ((byte) 0x7C)
#define ST25R_MOD_INDEX_30_8       ((byte) 0x7D)
#define ST25R_MOD_INDEX_31_2       ((byte) 0x7E)
#define ST25R_MOD_INDEX_31_6       ((byte) 0x7F)
#define ST25R_MOD_INDEX_31_9       ((byte) 0x80)
#define ST25R_MOD_INDEX_32_3       ((byte) 0x81)
#define ST25R_MOD_INDEX_32_6       ((byte) 0x82)
#define ST25R_MOD_INDEX_33_0       ((byte) 0x83)

#define ST25R_MOD_INDEX_MASK       ((byte) 0x7F)

/* Initiator operation modes */
#define ST25R_NFCIP1_OPMODE      ((byte) 0x00)
#define ST25R_ISO14443A_OPMODE   ((byte) 0x01)
#define ST25R_ISO14443B_OPMODE   ((byte) 0x02)
#define ST25R_FELICA_OPMODE      ((byte) 0x03)
#define ST25R_NFC_FORUM_OPMODE   ((byte) 0x04)
#define ST25R_SUBCARRIER_OPMODE  ((byte) 0x0E)
#define ST25R_BPSK_OPMODE        ((byte) 0x0F)

#define ST25R_OPMODE_POS         3
#define ST25R_OPMODE_MASK       ((byte) 0x78)

/* Tx/Rx constants */
#define TX_CRC_ON  ((byte) 0x00)
#define TX_CRC_OFF ((byte) 0x01)

/* Maximum constants */
#define MAX_TX_FIFO_LEN 96
#define MAX_RX_FIFO_LEN 96
#define MAX_SPI_TIMEOUT 10
#define MAX_ADDR_NUMBER 0x3F
#define MAX_NUMBER_TX_BYTES 8191

/* Bit Rates */
#define ST25R_106KBPS    ((byte) 0x00)
#define ST25R_212KBPS    ((byte) 0x01)
#define ST25R_424KBPS    ((byte) 0x02)
#define ST25R_848KBPS    ((byte) 0x03)
#define ST25R_1695KBPS   ((byte) 0x04)
#define ST25R_3390KBPS   ((byte) 0x05)
#define ST25R_6780KBPS   ((byte) 0x06)

#define ST25R_TX_106KBPS    ((byte) 0x00 << 4)
#define ST25R_TX_212KBPS    ((byte) 0x01 << 4)
#define ST25R_TX_424KBPS    ((byte) 0x02 << 4)
#define ST25R_TX_848KBPS    ((byte) 0x03 << 4)
#define ST25R_TX_1695KBPS   ((byte) 0x04 << 4) /* Supported only for ISO14443-B mode */
#define ST25R_TX_3390KBPS   ((byte) 0x05 << 4) /* Supported only for ISO14443-B mode */
#define ST25R_TX_6780KBPS   ((byte) 0x06 << 4) /* Supported only for ISO14443-B mode */

#define ST25R_RX_106KBPS    ((byte) 0x00)
#define ST25R_RX_212KBPS    ((byte) 0x01)
#define ST25R_RX_424KBPS    ((byte) 0x02)
#define ST25R_RX_848KBPS    ((byte) 0x03)
#define ST25R_RX_1695KBPS   ((byte) 0x04)
#define ST25R_RX_3390KBPS   ((byte) 0x05)

/* Protocol type */
#define ST25R_PROTOCOL_ISO14443A    ((byte) 0x00)
#define ST25R_PROTOCOL_ISO14443B    ((byte) 0x01)

/* Interrupt Requests Reasons */
typedef enum irq_reason {
    ST25R_NO_IRQ = 0,
    ST25R_OSC_FREQ_STABLE_IRQ,
    ST25R_FIFO_WATER_LVL_IRQ,
    ST25R_RX_START_IRQ,
    ST25R_RX_STOP_IRQ,
    ST25R_TX_STOP_IRQ,
    ST25R_BIT_COLL_IRQ,
    ST25R_DIRECT_CMD_STOP_IRQ,
    ST25R_NO_RESP_TIM_IRQ,
    ST25R_GP_TIM_IRQ,
    ST25R_EXT_FIELD_HIGH_IRQ,
    ST25R_EXT_FIELD_LOW_IRQ,
    ST25R_COLL_DURING_RF_COLL_AVOIDANCE_IRQ,
    ST25R_MIN_GUARD_TIME_VIOL_IRQ,
    ST25R_INITIATOR_RECV_IN_TARGET_MODE_IRQ,
    ST25R_CRC_ERROR_IRQ,
    ST25R_PARITY_ERROR_IRQ,
    ST25R_SOFT_FRAMING_ERROR_IRQ,
    ST25R_HARD_FRAMING_ERROR_IRQ,
    ST25R_WUP_TIM_IRQ,
    ST25R_WUP_TIM_AMP_MEAS_IRQ,
    ST25R_WUP_TIM_PHASE_MEAS_IRQ,
    ST25R_WUP_TIM_CAP_MEAS_IRQ
} irq_reason_t;



/* Function Prototypes -------------------------------------- */


/* Basic functions */
byte ST25R_WriteRegister(byte, byte);
byte ST25R_ReadRegister(byte, byte *);
byte ST25R_WriteMultipleRegisters(byte, byte *, byte);
byte ST25R_ReadMultipleRegisters(byte, byte *, byte);
byte ST25R_LoadFifo(byte *, byte);
byte ST25R_ReadFifo(byte *, byte *);
byte ST25R_SendDirectCommand(byte);
byte ST25R_SetBitInRegister(byte, byte);
byte ST25R_ClearBitInRegister(byte, byte);
byte ST25R_ModifyRegister(byte, byte, byte);

/* Initialization and Configuration */
byte ST25R_CheckIC(void);
byte ST25R_InitModule(void);
byte ST25R_InitReaderOperations(void);
byte ST25R_SetBitRate(byte);
byte ST25R_SelectTypeA(void);
byte ST25R_SelectTypeB(void);

/* ISO14443A Tx/Rx functions */
byte ST25R_TransmitREQA(void);
byte ST25R_TransmitWUPA(void);
byte ST25R_PerformAnticollA(void);
byte ST25R_SendAPDU(byte *, byte);
byte ST25R_ReceiveAPDU(byte *, ushort *);


#endif /* INC_ST25R3911B_HAL_DRIVER_H_ */


