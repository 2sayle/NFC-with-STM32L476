/*
 * ST25R3911B_errors.h
 *
 *  Created on: Jun 2, 2024
 *      Author: Elyass Jaoudat
 */


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


#endif /* INC_ST25R3911B_ERRORS_H_ */