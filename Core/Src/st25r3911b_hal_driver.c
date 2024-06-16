/**
 * @file st25r3911b_hal_driver.c
 * @brief This file contains the HAL driver implementation for the ST25R3911B NFC controller.
 *
 * This file provides the necessary functions to initialize and control the ST25R3911B NFC controller
 * using the HAL (Hardware Abstraction Layer) library provided by STMicroelectronics.
 *
 * @date Jun 2, 2024
 * @author Elyass Jaoudat
 */

/* Includes ------------------------------------------------- */
#include <string.h>
#include "stm32l476xx.h"
#include "main.h"
#include "macros.h"
#include "st25r3911b_hal_driver.h"
#include "st25r3911b_errors.h"


/* Exported variables --------------------------------------- */

/* Hardware */
byte IC_Version = 0;

/* SPI */
extern SPI_HandleTypeDef hspi1;

/* IRQ Handler */
extern byte ST25R_IRQ_FLAG;
extern irq_reason_t ST25R_IRQ_REASON;

/* FIFO */
byte fifo_lb = 0;
byte fifo_ncp = 0;
byte fifo_np_lb = 0;

/* Private functions prototypes ----------------------------- */

static byte WaitForSpecificIRQ(irq_reason_t irqReason);
static byte WaitForDirectCommand(void);


/* Static functions ----------------------------------------- */

static byte WaitForSpecificIRQ(irq_reason_t irqReason) {
    /* Wait for the IRQ to happen */
    while (ST25R_IRQ_FLAG == 0);

    /* Check if correct IRQ reason has been set */
    if (ST25R_IRQ_REASON != irqReason) {
        return ST25R_IRQ_ERROR;
    }
    /* Clear the IRQ flag */
    ST25R_IRQ_FLAG = 0;

    return ST25R_OK;
}

static byte WaitForDirectCommand(void) {
    /* Wait for the IRQ to happen */
    while (ST25R_IRQ_FLAG == 0);

    /* Check if correct IRQ reason has been set */
    if (ST25R_IRQ_REASON != ST25R_DIRECT_CMD_STOP_IRQ) {
        return ST25R_IRQ_ERROR;
    }
    /* Clear the IRQ flag */
    ST25R_IRQ_FLAG = 0;

    return ST25R_OK;
}


/* Functions ------------------------------------------------ */


/**
 * @brief Write a register of the ST25R3911B
 * 
 * @param addr Address of the register to be written
 * @param regData Data to be written in the register
 * @return byte ST25R_OK if the register has been written successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
/* Basic functions */
byte ST25R_WriteRegister(byte addr, byte regData) {

    /* Assert parameters */
    if (addr > MAX_ADDR_NUMBER) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Add appropriate mask (not used in write reg mode) */
    addr |= ST25R_REG_WRITE;

    /* Send address */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    
    /* Send data to modify associated register */
    if (HAL_SPI_Transmit(&hspi1, &regData, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;

}


/**
 * @brief Read a register of the ST25R3911B
 * 
 * @param addr Address of the register to be read
 * @param regData Pointer to the register data
 * @return byte ST25R_OK if the register has been read successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */

byte ST25R_ReadRegister(byte addr, byte *regData) {
    /* Assert parameters */
    if (addr > MAX_ADDR_NUMBER) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Add appropriate mask */
    addr |= ST25R_REG_READ;
    
    /* Send address */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, &addr, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Receive register data */
    if (HAL_SPI_Receive(&hspi1, regData, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    SPI1_SET_NSS();
    return ST25R_OK;

}

/**
 * @brief Write multiple registers of the ST25R3911B
 * 
 * @param startAddr Start address of the registers to be written
 * @param regDataArray Pointer to the data buffer
 * @param len Length of the data buffer
 * @return byte ST25R_OK if the registers have been written successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
byte ST25R_WriteMultipleRegisters(byte startAddr, byte *regDataArray, byte len) {

    /* Assert parameters */
    if ((startAddr > MAX_ADDR_NUMBER) || (regDataArray == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Prepare buffer */
    byte txBuf[MAX_ADDR_NUMBER + 1] = {0};
    txBuf[0] = startAddr | ST25R_REG_WRITE;
    memcpy(&txBuf[1], regDataArray, (size_t)len);

    /* Send data stream to the FIFO */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, txBuf, len+1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;
}


/**
 * @brief Read multiple registers of the ST25R3911B
 * 
 * @param startAddr Start address of the registers to be read
 * @param regDataArray Pointer to the data buffer
 * @param len Length of the data buffer
 * @return byte ST25R_OK if the registers have been read successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */

byte ST25R_ReadMultipleRegisters(byte startAddr, byte *regDataArray, byte len) {

    /* Assert parameters */
    if ((startAddr > MAX_ADDR_NUMBER) || (regDataArray == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Prepare buffer */
    byte txBuf[MAX_ADDR_NUMBER + 1] = {0};
    txBuf[0] = startAddr | ST25R_REG_READ;

    /* Send data stream to the FIFO */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, txBuf, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Receive data stream */
    if (HAL_SPI_Receive(&hspi1, regDataArray, len, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;

}


/**
 * @brief Load the FIFO of the ST25R3911B
 * 
 * @param pData Pointer to the data buffer
 * @param len Length of the data buffer
 * @return byte ST25R_OK if the FIFO has been loaded successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */

byte ST25R_LoadFifo(byte *pData, byte len) {

    /* Local variables */
    byte txBuf[MAX_TX_FIFO_LEN + 1] = {0};

    /* Assert parameters */
    if ((pData == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }
    if (len > MAX_TX_FIFO_LEN) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Prepare buffer */
    txBuf[0] = ST25R_FIFO_LOAD;
    memcpy(&txBuf[1], pData, (size_t)len);

    /* Send data stream to the FIFO */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, txBuf, len+1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;

}


/**
 * @brief Read the FIFO of the ST25R3911B
 * 
 * @param pData Pointer to the data buffer
 * @param len Pointer to the length of the data buffer
 * @return byte ST25R_OK if the FIFO has been read successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_FIFO_EMPTY_ERROR if the FIFO is empty
 * @return byte ST25R_FIFO_UDFLOW_ERROR if an underflow occurs
 * @return byte ST25R_FIFO_OVFLOW_ERROR if an overflow occurs
 * @return byte ST25R_PARITY_FRAMING_ERROR if a parity or framing error occurs
 */

byte ST25R_ReadFifo(byte *pData, byte *len) {

    /* Local variables */
    byte regData = 0;
    byte fifoLen = 0;
    byte flag = ST25R_FIFO_READ;

    /* Assert parameters */
    if ((pData == NULL) || (*len == 0) || (len == NULL)) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Get number of bytes to be read in FIFO */
    if (ST25R_ReadRegister(ST25R_FIFO1_STATUS_REG, &fifoLen) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Check FIFO status register 1 */
    if (fifoLen == 0x00) {
        return ST25R_FIFO_EMPTY_ERROR;
    }
    if (fifoLen > MAX_RX_FIFO_LEN) {
        return ST25R_UNKNOWN_ERROR;
    }

    /* Check FIFO status register 2 */
    if (ST25R_ReadRegister(ST25R_FIFO1_STATUS_REG, &regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (regData & 0x40) {
        return ST25R_FIFO_UDFLOW_ERROR;
    }
    if (regData & 0x20) {
        return ST25R_FIFO_OVFLOW_ERROR;
    }

    /* Store info */
    fifo_np_lb = regData & 0x01;
    if (fifo_np_lb) {
        return ST25R_PARITY_FRAMING_ERROR;
    }

    fifo_ncp = (regData & 0x10) >> 4;

    /* FIFO_NOT_COMPLETE flag */
    if (fifo_ncp) {
        fifo_lb = (regData & 0x0E) >> 1;
    }
    else {
        fifo_lb = 0;
    }

    /* Send read fifo flag */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, &flag, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    
    /* Read fifo */
    if (HAL_SPI_Receive(&hspi1, pData, (uint16_t)fifoLen, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;

}

/**
 * @brief Send a direct command to the ST25R3911B
 * 
 * @param cmd Command to be sent
 * @return byte ST25R_OK if the command has been sent successfully
 * @return byte ST25R_WRONG_PARAMETER if the command is not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */

byte ST25R_SendDirectCommand(byte cmd) {
    
    /* Assert parameter */
    if (((cmd > 0xE3)|| (cmd < 0xC1)) && (cmd != 0xFC) && (cmd != 0xD2))  {
        return ST25R_WRONG_PARAMETER;
    }

    /* Add appropriate mask */
    cmd |= ST25R_DIRECT_CMD;

    /* Send command */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, &cmd, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    return ST25R_OK;
}


/**
 * @brief Set a bit in a register of the ST25R3911B
 * 
 * @param addr Address of the register
 * @param pos Position of the bit to be set
 * @return byte ST25R_OK if the bit has been set successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 */

byte ST25R_SetBitInRegister(byte addr, byte pos) {

    /* Assert parameters */
    if (addr > MAX_ADDR_NUMBER) {
        return ST25R_WRONG_PARAMETER;
    }
    if (pos > 7) {
        return ST25R_WRONG_PARAMETER;
    }


    /* Local variables */
    byte regData = 0;

    /* Read register */
    if (ST25R_ReadRegister(addr, &regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Set bit */
    ST25R_SET_BIT(regData, pos);

    /* Write register */
    if (ST25R_WriteRegister(addr, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    return ST25R_OK;
}

/**
 * @brief Clears a bit in a register of the ST25R3911B
 * 
 * @param addr Address of the register
 * @param pos Position of the bit to be reset
 * @return byte ST25R_OK if the bit has been reset successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 *
 */
byte ST25R_ClearBitInRegister(byte addr, byte pos) {
    /* Assert parameters */
    if (addr > MAX_ADDR_NUMBER) {
        return ST25R_WRONG_PARAMETER;
    }
    if (pos > 7) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Local variables */
    byte regData = 0;

    /* Read register */
    if (ST25R_ReadRegister(addr, &regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Reset bit */
    ST25R_RESET_BIT(regData, pos);

    /* Write register */
    if (ST25R_WriteRegister(addr, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    return ST25R_OK;
}

/**
 * @brief Modify a register of the ST25R3911B
 * 
 * @param addr Address of the register
 * @param value Value to be written in the register
 * @param mask Mask to be applied to the register
 * @return byte ST25R_OK if the register has been modified successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
byte ST25R_ModifyRegister(byte addr, byte value, byte mask) {
    /* Local variables */
    byte regData = 0;

    /* Read register */
    if (ST25R_ReadRegister(addr, &regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Modify register */
    ST25R_MODIFY_REG(regData, value, mask);

    /* Write register */
    if (ST25R_WriteRegister(addr, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    return ST25R_OK;
}


/**
 * @brief Check the presence of the ST25R3911B
 * 
 * @return byte ST25R_OK if the ST25R3911B is present
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_CheckIC(void) {
    /* Local variables */
    byte status = ST25R_OK;
    byte regData = 0;

    /* Read the version register */
    status = ST25R_ReadRegister(ST25R_IC_IDENTITY_REG, &regData);
    if (status != ST25R_OK) {
        return status;
    }

    if ((regData & 0x08) != 0x08) {
        return ST25R_UNKNOWN_ERROR;
    }

    /* Store the IC version */
    IC_Version = regData & 0x07;

    return ST25R_OK;

}


/**
 * @brief Initialize the ST25R3911B module. It corresponds to the power-up sequence.
 * See section 1.2.16 of the ST25R3911B datasheet.
 * 
 * @return byte ST25R_OK if the module has been initialized successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
byte ST25R_InitModule(void) {

    /* Local variables */
    byte regData = 0;

    /* Power-Up Sequence (see section 1.2.16 of DS11793) */

    /* Configure the two IO configuration registers */
    regData = 0x3F;
    if (ST25R_WriteRegister(ST25R_IO_CONF1_REG, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    regData = 0x18;
    if (ST25R_WriteRegister(ST25R_IO_CONF2_REG, 0x00) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Set en bit in Operation control register (otherwise, regulator is OFF) */
    if (ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 7) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Configure regulator register */
    if (ST25R_WriteRegister(ST25R_REGUL_VOLTAGE_CTRL_REG, 0x00) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Send command to adjust the regulator */
    if (ST25R_SendDirectCommand(ST25R_ADJUST_REGULATORS_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (WaitForDirectCommand() != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    /* Calibrate the antenna (optional) */
    if (ST25R_SendDirectCommand(ST25R_CAL_ANTENNA_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (WaitForDirectCommand() != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    return ST25R_OK;

}

/**
 * @brief Initialize the reader operations
 * 
 * @return byte ST25R_OK if the reader operations have been initialized successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_InitReaderOperations(void) {

    /* Local variables */
    byte regData = 0;

    /* Configure Mode Definition register */
    ST25R_MODIFY_REG(regData, ST25R_ISO14443A_OPMODE, ST25R_OPMODE_MASK);
    if (ST25R_WriteRegister(ST25R_MODE_DEF_REG, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Configure Bit Rate Definition register */
    regData = ST25R_TX_106KBPS | ST25R_RX_106KBPS;
    if (ST25R_WriteRegister(ST25R_BIT_RATE_REG, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* (Optional) Configure other registers */

    /* Perform an analog preset command */
    if (ST25R_SendDirectCommand(ST25R_ANALOG_PRESET_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Enter in 'Ready mode' by setting 'en' bit in Operation Control register */
    if (ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 7) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (WaitForSpecificIRQ(ST25R_OSC_FREQ_STABLE_IRQ) != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    /* Set Tx/Rx enable bits in Operation Control register */
    if (ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 3) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 6) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* RFID protocols usually require that the reader field is turned on for a while before sending the first command (5 ms for ISO14443) */
    HAL_Delay(5);

    return ST25R_OK;

}


/**
 * @brief Set the bit rate of the ST25R3911B
 * 
 * @param bitRate Bit rate to be set
 * @return byte ST25R_OK if the bit rate has been set successfully
 * @return byte ST25R_WRONG_PARAMETER if the bit rate is not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
byte ST25R_SetBitRate(byte bitRate) {

    /* Local variables */
    byte regData = 0;

    /* Assert parameters */
    if (bitRate > ST25R_6780KBPS) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Modify the bit rate */
    switch (bitRate) {
        case ST25R_106KBPS:
            regData = ST25R_TX_106KBPS | ST25R_RX_106KBPS;
            break;
        case ST25R_212KBPS:
            regData = ST25R_TX_212KBPS | ST25R_RX_212KBPS;
            break;
        case ST25R_424KBPS:
            regData = ST25R_TX_424KBPS | ST25R_RX_424KBPS;
            break;
        case ST25R_848KBPS:
            regData = ST25R_TX_848KBPS | ST25R_RX_848KBPS;
            break;
        case ST25R_1695KBPS:
            regData = ST25R_TX_1695KBPS | ST25R_RX_1695KBPS;
            break;
        case ST25R_3390KBPS:
            regData = ST25R_TX_3390KBPS | ST25R_RX_3390KBPS;
            break;
        case ST25R_6780KBPS:
            regData = ST25R_TX_6780KBPS | ST25R_RX_3390KBPS;
            break;
        default:
            return ST25R_WRONG_PARAMETER;
    }

    /* Write the new bit rate */
    if (ST25R_WriteRegister(ST25R_BIT_RATE_REG, regData) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    return ST25R_OK;

}



/**
 * @brief Select the ISO14443 Type A protocol
 * 
 * @return byte ST25R_OK if the Type A protocol has been selected successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_IRQ_ERROR if an error occurs during IRQ handling
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_SelectTypeA(void) {

    /* Local variables */
    byte status = ST25R_OK;
    byte regData = 0;

    /* Configure Mode Definition register */
    status = ST25R_ModifyRegister(ST25R_MODE_DEF_REG, ST25R_ISO14443A_OPMODE << ST25R_OPMODE_POS, ST25R_OPMODE_MASK);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure ISO14443A bit rate register (106kbps by default) */
    regData = ST25R_TX_106KBPS | ST25R_RX_106KBPS;
    status = ST25R_WriteRegister(ST25R_BIT_RATE_REG, regData);
    if (status != ST25R_OK) {
        return status;
    }

    /* Set anticollision bit in proper register */
    status = ST25R_SetBitInRegister(ST25R_ISO14443A_NFC_REG, 0);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure Mask Receive Timer register to be compliant with ISO14443A Frame Delay Time (FDT) */
    regData = 17;  // MRT < FDT + 64/fc
    status = ST25R_WriteRegister(ST25R_MASK_RX_TIMER_REG, regData);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure No Response Timer register to be compliant with ISO14443A FDT */
    regData = 21; // NRT > FDT + 64/fc
    status = ST25R_WriteRegister(ST25R_NO_RESP_TIMER2_REG, regData);
    if (status != ST25R_OK) {
        return status;
    }

    /* (Optional) Configure other registers */

    /* Perform an analog preset command */
    status = ST25R_SendDirectCommand(ST25R_ANALOG_PRESET_CMD);
    if (status != ST25R_OK) {
        return status;
    }

    /* Enter in 'Ready mode' by setting 'en' bit in Operation Control register */
    status = ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 7);
    if (status != ST25R_OK) {
        return status;
    }
    if (WaitForSpecificIRQ(ST25R_OSC_FREQ_STABLE_IRQ) != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    /* Set Tx/Rx enable bits in Operation Control register */
    status = ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 3);
    if (status != ST25R_OK) {
        return status;
    }
    status = ST25R_SetBitInRegister(ST25R_OP_CONTROL_REG, 6);
    if (status != ST25R_OK) {
        return status;
    }

    /* RFID protocols usually require that the reader field is turned on for a while before sending the first command (5 ms for ISO14443) */
    HAL_Delay(5);

    return ST25R_OK;

}


/**
 * @brief Select the ISO14443 Type B protocol
 * 
 * @return byte ST25R_OK if the Type B protocol has been selected successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 */
byte ST25R_SelectTypeB(void) {
    
    /* Local variables */
    byte modIndex = ST25R_MOD_INDEX_9_9 & ST25R_MOD_INDEX_MASK;

    /* Set modulation index in AM modulation register */
    if (ST25R_WriteRegister(ST25R_AM_MOD_DEPTH_CONTROL_REG, modIndex) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Send 'Calibrate modulation depth' direct command */
    if (ST25R_SendDirectCommand(ST25R_CAL_MOD_DEPTH_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (WaitForDirectCommand() != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }



    
    return ST25R_OK;

}


/* ISO14443A functions */

/**
 * @brief Transmit a REQA command to the ST25R3911B
 * 
 * @note This function is used to initiate a communication with a PICC (Proximity Integrated Circuit Card) in ISO14443A mode.
 * 
 * @return byte ST25R_OK if the REQA command has been transmitted successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_IRQ_ERROR if an error occurs during IRQ handling
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_TransmitREQA(void) {

    /* Local variables */
    byte status = ST25R_OK;
    byte regData = 0;

    /* Set no_crc_rx bit in Auxiliary Definition register */
    status = ST25R_SetBitInRegister(ST25R_AUX_DEF_REG, 7);
    if (status != ST25R_OK) {
        return status;
    }

    /* Send the REQA command */
    status = ST25R_SendDirectCommand(ST25R_TRANSMIT_REQA_CMD);
    if (status != ST25R_OK) {
        return status;
    }
    if (WaitForDirectCommand() != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }
    /* Wait for the response */
    if (WaitForSpecificIRQ(ST25R_TX_STOP_IRQ) != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    /* Read the FIFO */
    status = ST25R_ReadFifo(&regData, &fifo_lb);
    if (status != ST25R_OK) {
        return status;
    }

    /* Check the response */
    if (regData != 0x04) {
        return ST25R_UNKNOWN_ERROR;
    }

    return status;
}

byte ST25R_TransmitWUPA(void);
byte ST25R_PerformAnticollA(void);

/* ISO14443B functions */

/* Generic transfer functions */

/**
 * @brief Transmit data to the PICC
 * 
 * @note This function is used to transmit bytes to the PICC 
 * regardless of the selected protocol.
 * 
 * @param pData Pointer to the data buffer
 * @param len Length of the data buffer
 * @param timeout Timeout for the transmission
 * 
 * @return byte ST25R_OK if the data has been transmitted successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 * @return byte ST25R_FIFO_OVFLOW_ERROR if an overflow occurs
 * @return byte ST25R_FIFO_UDFLOW_ERROR if an underflow occurs
 * @return byte ST25R_FIFO_EMPTY_ERROR if the FIFO is empty
 */
byte ST25R_TransmitData(byte *pData, ushort len, word timeout, byte crcOption) {

    /* Assert parameters */
    if ((pData == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }
    if (len > MAX_NUMBER_TX_BYTES) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Local variables */
    byte status = ST25R_OK;
    byte regData = 0;
    byte msb = 0;
    byte lsb = 0;

    /* Send 'Clear' direct command to clear the FIFO */
    status = ST25R_SendDirectCommand(ST25R_CLEAR_CMD);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure the Number of Transmitted Bytes registers 1 and 2 */
    len <<= 3;
    msb = (byte) ((len & 0xFF00) >> 8);
    lsb = (byte) (len & 0x00FF);
    status = ST25R_WriteRegister(ST25R_NB1_TX_BYTES_REG, msb);
    if (status != ST25R_OK) {
        return status;
    }
    status = ST25R_WriteRegister(ST25R_NB2_TX_BYTES_REG, lsb);
    if (status != ST25R_OK) {
        return status;
    }


    /* Case 1. Data length is inferior to FIFO length */
    if (len <= MAX_TX_FIFO_LEN) {

        /* Fill the FIFO */
        status = ST25R_LoadFifo(pData, (byte)len);
        if (status != ST25R_OK) {
            return status;
        }

        /* Send the data */
        if (crcOption == TX_CRC_ON) {
            status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITH_CRC_CMD);
        } else {
            status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITHOUT_CRC_CMD);
        }
        if (status != ST25R_OK) {
            return status;
        }

        /* Wait for the end of transmission */
        if (WaitForSpecificIRQ(ST25R_TX_STOP_IRQ) != ST25R_OK) {
            return ST25R_IRQ_ERROR;
        }

    }

    /* Case 2. Data length is superior to FIFO length */
    else {

        /* Compute the number of iterations */
        byte nbFifoLoads = len % MAX_TX_FIFO_LEN;
        byte nbLeftBytes = len - nbFifoLoads;

        for (byte i = 0; i < nbFifoLoads; i++) {

            /* Fill the FIFO */
            status = ST25R_LoadFifo(pData, MAX_TX_FIFO_LEN);
            if (status != ST25R_OK) {
                return status;
            }

            /* Send the data */
            if (crcOption == TX_CRC_ON) {
                status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITH_CRC_CMD);
            } else {
                status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITHOUT_CRC_CMD);
            }
            if (status != ST25R_OK) {
                return status;
            }

            /* Wait for the water level IRQ */
            if (WaitForSpecificIRQ(ST25R_FIFO_WATER_LVL_IRQ) != ST25R_OK) {
                return ST25R_IRQ_ERROR;
            }

            /* Update the data pointer */
            pData += MAX_TX_FIFO_LEN;

        }

        /* Fill the FIFO with the remaining bytes */
        status = ST25R_LoadFifo(pData, nbLeftBytes);
        if (status != ST25R_OK) {
            return status;
        }

        /* Send the data */
        if (crcOption == TX_CRC_ON) {
            status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITH_CRC_CMD);
        } else {
            status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WITHOUT_CRC_CMD);
        }

        /* Wait for the end of transmission */
        if (WaitForSpecificIRQ(ST25R_TX_STOP_IRQ) != ST25R_OK) {
            return ST25R_IRQ_ERROR;
        }

    }

    return ST25R_OK;

}
byte ST25R_ReceiveData(byte *pData, byte *len, ushort timeout);


byte ST25R_SendAPDU(byte *apdu, byte len);
byte ST25R_ReceiveAPDU(byte *apdu, ushort *len);




