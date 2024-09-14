/**
 * @file iso14443.c 
 * @author Elyass Jaoudat
 * @brief This file contains the implementation of ISO/IEC 14443-4 specification.
 * 
 * @date Sep 13, 2024
 *
 */

/* Includes ------------------------------------------------- */
#include <string.h>
#include "main.h"
#include "macros.h"
#include "iso14443.h"
#include "st25r3911b_hal_driver.h"
#include "st25r3911b_errors.h"


/* Global variables ----------------------------------------- */
byte fsci;
byte fsdi;
ats_t ats;
byte cid;
byte txBuffer[FSC0+2];
byte rxBuffer[FSD0+2];


/* Private functions prototypes ----------------------------- */
static byte ISO_ST25R_ErrorDispatcher(byte);
static byte ISO_CheckPCB(byte);
static byte ISO_CheckCRC(byte *, word);
/* Private functions ---------------------------------------- */

static byte ISO_ST25R_ErrorDispatcher(byte status) {
    switch (status) {
        case ST25R_OK:
            return ISO_SUCCESS;
        case ST25R_FIFO_OVFLOW_ERROR:
            return ISO_ERROR_LEN;
        case ST25R_FIFO_UDFLOW_ERROR:
            return ISO_ERROR_LEN;
        case ST25R_FIFO_EMPTY_ERROR:
            return ISO_ERROR_NO_RESPONSE;
        case ST25R_PARITY_FRAMING_ERROR:
            return ISO_ERROR_PARITY;
        case ST25R_CRC_ERROR:
            return ISO_ERROR_CRC;
        case ST25R_TIMEOUT_ERROR:
            return ISO_ERROR_TIMEOUT;
        case ST25R_NO_RESP_ERROR:
            return ISO_ERROR_NO_RESPONSE;
        case ST25R_WRONG_PARAMETER:
        case ST25R_SPI_IO_STREAM_ERROR:
        case ST25R_UNKNOWN_ERROR:
        case ST25R_IRQ_ERROR:
        case ST25R_PWR_SUPPLY_ERROR:
        default:
            return ISO_ERROR;
    }
}

static byte ISO_CheckPCB(byte pcb) {
    if ((pcb & I_BLOCK_MASK) == I_BLOCK_PATTERN) {
        return ISO_SUCCESS;
    } else if ((pcb & R_BLOCK_MASK) == R_BLOCK_PATTERN) {
        return ISO_SUCCESS;
    } else if ((pcb & S_BLOCK_MASK) == S_BLOCK_PATTERN) {
        return ISO_SUCCESS;
    } else {
        return ISO_ERROR;
    }
}

static byte ISO_CheckCRC(byte *buffer, word len) {
    ushort crc16 = ISO_ComputeCRC16(buffer, len);
    if (MSB(crc16) != buffer[len-1] || LSB(crc16) != buffer[len])
        return ISO_ERROR_CRC;
    else {
        return ISO_SUCCESS;
    }
}

static word ISO_ComputeSFGT(byte sfgi) {
    return (word) (256 * (1 << sfgi) * 16 * FC_INV_MS);
}
static word ISO_ComputeFWT(byte fwi) {
    return (word) (256 * (1 << fwi) * 16 * FC_INV_MS);
}


/* Functions ------------------------------------------------ */


/**
 * @brief Perform the RATS command with the PICC.
 * 
 * @param ratsResp Pointer to the RATS response buffer.
 * @param recvLen Length of the RATS response buffer.
 * @return byte ISO_SUCCESS: RATS command successful.
 * @return byte ISO_ERROR: RATS command failed.
 * @return byte ISO_ERROR_LEN: Invalid length.
 * @return byte ISO_ERROR_CRC: CRC check failed.
 * @return byte ISO_ERROR_NO_RESPONSE: No response from PICC.
 * @return byte ISO_ERROR_TIMEOUT: Timeout error.
 * @return byte ISO_ERROR_PARITY: Parity error.
 */
byte ISO_PerformRATS(byte *ratsResp, ushort *recvLen) {

    /* Local variables */
    byte rats[2] = {RATS, 0x00};
    byte status = ST25R_OK;

    /* FSDI and CID */
    rats[1] = (byte) (FSDI0 << 4);  // FSDI = 0x08 (default)
    rats[1] |= 0x00;                // CID = 0x00

    /* Send RATS command */
    status = ST25R_TransmitData(rats, 2, TX_CRC_ON);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    /* Receive RATS response */
    status = ST25R_ReceiveData(ratsResp, recvLen);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    /* Check response */
    status = ISO_StoreATS(ratsResp, (byte) *recvLen, &ats);

    return status;
}

/**
 * @brief Analyzes and stores the Answer-To-Select provided by the PICC.
 * 
 * @param buffer Pointer to the RATS response buffer. 
 * @param len Length of the RATS response buffer.
 * @param ats Structure to store the ATS parameters.
 * @return byte ISO_SUCCESS: ATS stored successfully.
 * @return byte ISO_ERROR: ATS storage failed.
 * @return byte ISO_ERROR_LEN: Invalid length.
 * @return byte ISO_ERROR_CRC: CRC check failed.
 * @return byte ISO_ERROR_PARITY: Parity error.
 */
byte ISO_StoreATS(byte *buffer, byte len, ats_t *ats) {

    /* Assert parameters */
    if (buffer == NULL || ats == NULL)
        return ISO_ERROR;
    if (len == 0 || len > ATS_MAX_LEN)
        return ISO_ERROR_LEN;
    if (ISO_CheckCRC(buffer, (word) len) != ISO_SUCCESS)
        return ISO_ERROR_CRC;

    /* Local variables */
    byte pos = 0;
    
    /* Check TL */
    if (buffer[0] + 2 != len) {
        return ISO_ERROR_LEN;
    } else {
        ats->TL = buffer[pos++];
    }

    /* Store format byte T0 and interface bytes : TA1, TB1, TC1*/
    ats->T0 = buffer[pos++];
    fsci = ats->T0 & 0x0F;
    if (ats->T0 & 0x10)
        ats->TA1 = buffer[pos++];
    if (ats->T0 & 0x20)
        ats->TB1 = buffer[pos++];
    if (ats->T0 & 0x40)
        ats->TC1 = buffer[pos++];

    /* Store historical bytes */
    ats->NbHist = len-pos-2;
    for (int i = 0; i < len-pos-2; i++) {
        ats->TH[i] = buffer[pos+i];
    }

    ats->atsBuffer = buffer;

    return ISO_SUCCESS;

}

/**
 * @brief Perform the PPS exchange with the PICC.
 * 
 * @param pps Pointer to the PPS buffer.
 * @param len Length of the PPS buffer.
 * @return byte ISO_SUCCESS: PPS exchange successful.
 * @return byte ISO_ERROR: PPS exchange failed.
 * @return byte ISO_ERROR_LEN: Invalid length.
 * @return byte ISO_ERROR_CRC: CRC check failed.
 */
byte ISO_ExchangePPS(byte *pps, byte len) {

    /* Assert parameters */
    if (pps == NULL) { return ISO_ERROR;}
    if (len != 4 || len != 5) { return ISO_ERROR_LEN; }
    if (ISO_CheckCRC(pps, (word)len) != ISO_SUCCESS) {
        return ISO_ERROR_CRC;
    }

    /* Local variables */
    byte status = ST25R_OK;
    byte rxLen = 0;
    byte ppsResp[5] = {0};

    /* Send PPS request */
    status = ST25R_TransmitData(pps, len, TX_CRC_ON);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    /* Receive PPS response */
    status = ST25R_ReceiveData(ppsResp, (ushort *) &rxLen);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }
    
    /* Check response */
    if (rxLen != 3) {
        return ISO_ERROR_LEN;
    }
    if (ISO_CheckCRC(pps, (word)rxLen) != ISO_SUCCESS) {
        return ISO_ERROR_CRC;
    }
    if (pps[0] != ppsResp[0]) {
        return ISO_ERROR;
    }

    return ISO_SUCCESS;
}

/**
 * @brief Send a block of data to the PICC according to ISO14443-4.
 *        It prepares the block by adding the PCB and CRC bytes (mandatory)
 *        and CID, NAD bytes (optional).
 * 
 * @param buffer Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @param pcb PCB byte.
 * @return byte ISO_SUCCESS: Block sent successfully.
 * @return byte ISO_ERROR: Block sending failed.
 * @return byte ISO_ERROR_LEN: Invalid length.
 */
byte ISO_SendBlock(byte *buffer, ushort len, byte pcb) {
    
    /* Assert parameters */
    if (buffer == NULL) { return ISO_ERROR; }
    if (len == 0 || len > (FSC0 - 5)) { return ISO_ERROR_LEN; }

    /* Local variables */
    byte status = ST25R_OK;

    /* Check PCB */
    if (ISO_CheckPCB(pcb) != ISO_SUCCESS) {
        return ISO_ERROR;
    }
    /* Prepare buffer */
    txBuffer[0] = pcb;
    if (pcb & CID_MASK) {
        txBuffer[1] = cid;
        memmove(&txBuffer[2], buffer, (size_t)len);
    } else {
        memmove(&txBuffer[1], buffer, (size_t)len);
    }

    /* Send block */
    status = ST25R_TransmitData(txBuffer, len+1, TX_CRC_ON);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    return ISO_SUCCESS;

}


/**
 * @brief Receive a block of data from the PICC according to ISO14443-4.
 *        It receives the block and extracts the PCB, CID, NAD, and data bytes.
 * 
 * @param buffer Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @return byte ISO_SUCCESS: Block received successfully.
 * @return byte ISO_ERROR: Block receiving failed.
 * @return byte ISO_ERROR_LEN: Invalid length.
 * @return byte ISO_ERROR_CRC: CRC check failed.
 * @return byte ISO_ERROR_PARITY: Parity error.
 */
byte ISO_ReceiveBlock(byte *buffer, ushort *len) {

    /* Assert parameters */
    if (buffer == NULL) { return ISO_ERROR; }
    if (len == NULL || *len == 0 || *len > FSD0) { return ISO_ERROR_LEN; }

    /* Local variables */
    byte status = ST25R_OK;
    byte pcb = 0;

    /* Receive block */
    status = ST25R_ReceiveData(rxBuffer, len);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    /* Check CRC */
    if (ISO_CheckCRC(rxBuffer, (word)*len) != ISO_SUCCESS) {
        return ISO_ERROR_CRC;
    }

    /* Check PCB */
    pcb = rxBuffer[0];
    if (ISO_CheckPCB(pcb) != ISO_SUCCESS) {
        return ISO_ERROR;
    }

    /* Extract CID */
    if (pcb & CID_MASK) {
        cid = rxBuffer[1];
        memmove(buffer, &rxBuffer[2], (size_t)*len-2);
        *len -= 2;
    } else {
        memmove(buffer, &rxBuffer[1], (size_t)*len-1);
        *len -= 1;
    }

    return ISO_SUCCESS;
}


byte ISO_ExchangeAPDU(byte *apdu, ushort len) {

    /* Assert parameters */
    if (apdu == NULL) { return ISO_ERROR; }
    if (len == 0 || len > FSC0) { return ISO_ERROR_LEN; }

    /* Local variables */
    byte status = ST25R_OK;
    byte rxLen = 0;
    byte apduResp[FSC0] = {0};

    /* Send APDU command */
    status = ST25R_TransmitData(apdu, len, TX_CRC_ON);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    /* Receive APDU response */
    status = ST25R_ReceiveData(apduResp, (ushort *) &rxLen);
    if (status != ST25R_OK) {
        return ISO_ST25R_ErrorDispatcher(status);
    }

    return ISO_SUCCESS;
}