/**
 * @file st25r3911b_errors.h
 * @brief Header file containing definitions of error cases.
 * @date Jun 3, 2024
 * @author Elyass Jaoudat
 */


#pragma once

#ifndef INC_ST25R3911B_ERRORS_H_
#define INC_ST25R3911B_ERRORS_H_


#define ST25R_OK                    ((byte) 0x00)
#define ST25R_WRONG_PARAMETER       ((byte) 0x01)
#define ST25R_SPI_IO_STREAM_ERROR   ((byte) 0x02)
#define ST25R_UNKNOWN_ERROR         ((byte) 0x03)
#define ST25R_FIFO_OVFLOW_ERROR     ((byte) 0x04)
#define ST25R_FIFO_UDFLOW_ERROR     ((byte) 0x05)
#define ST25R_FIFO_EMPTY_ERROR      ((byte) 0x06)
#define ST25R_PARITY_FRAMING_ERROR  ((byte) 0x07)   
#define ST25R_IRQ_ERROR             ((byte) 0x08)
#define ST25R_PWR_SUPPLY_ERROR      ((byte) 0x09)
#define ST25R_CRC_ERROR             ((byte) 0x0A)
#define ST25R_TIMEOUT_ERROR         ((byte) 0x0B)


#endif /* INC_ST25R3911B_ERRORS_H_ */