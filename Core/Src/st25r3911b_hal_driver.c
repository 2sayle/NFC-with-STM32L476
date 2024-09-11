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
#include "iso14443.h"
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
byte rxBuffer[MAX_NUMBER_RX_BYTES] = {0};
byte txBuffer[MAX_NUMBER_TX_BYTES] = {0};

/* ISO14443 */
pcd_state ST25R_PCD_STATE = PCD_STANDBY;

/* Private functions prototypes ----------------------------- */

static byte WaitForSpecificIRQ(irq_reason_t);
static byte WaitForSpecificIRQ_T(irq_reason_t, word);
static byte WaitForDirectCommand(void);
static byte CheckForSpecificIRQ(irq_reason_t);


/* Static functions ----------------------------------------- */


/**
 * @brief Wait for an IRQ to happen (blocking)
 * 
 * @param None
 * @return irq_reason_t 
 */
static irq_reason_t WaitForIRQ(void) {
    /* Wait for the IRQ to happen */
    while (ST25R_IRQ_FLAG == 0);

    /* Clear the IRQ flag */
    ST25R_IRQ_FLAG = 0;

    return ST25R_IRQ_REASON;
}

/**
 * @brief Wait for an IRQ to happen with a timeout in ms
 * 
 * @param timeout Value in ms
 * @return irq_reason_t 
 */
static irq_reason_t WaitForIRQ_T(word timeout) {
    /* Get the current tick value */
    word startTick = HAL_GetTick();

    /* Wait for the IRQ to happen */
    while (1) {
        /* Check if the timeout has elapsed */
        if ((HAL_GetTick() - startTick) > timeout) {
            return ST25R_TIMEOUT_ERROR;
        }

        if (ST25R_IRQ_FLAG == 1) {
            /* Clear the IRQ flag */
            ST25R_IRQ_FLAG = 0;
            return ST25R_IRQ_REASON;
        }
    }

    return ST25R_IRQ_ERROR;
}

/**
 * @brief Wait for a specific IRQ to happen (blocking)
 * 
 * @param irqReason
 * @return byte Return values:
 *         - ST25R_OK if the IRQ has been set
 *         - ST25R_IRQ_ERROR if an error occurs during IRQ handling
 */
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

/**
 * @brief Wait for a specific IRQ to happen with a timeout in ms
 * 
 * @param irqReason Number of the IRQ to wait for
 * @param timeout Value in ms
 * @return byte Return values:
 *         - ST25R_OK if the IRQ has been set
 *         - ST25R_TIMEOUT_ERROR if the timeout has elapsed
 *         - ST25R_IRQ_ERROR if an error occurs during IRQ handling
 */
static byte WaitForSpecificIRQ_T(irq_reason_t irqReason, word timeout) {
    /* Get the current tick value */
    word startTick = HAL_GetTick();

    /* Wait for the IRQ to happen */
    while (1) {
        /* Check if the timeout has elapsed */
        if ((HAL_GetTick() - startTick) > timeout) {
            return ST25R_TIMEOUT_ERROR;
        }

        if (ST25R_IRQ_FLAG == 1) {
            if (ST25R_IRQ_REASON == irqReason) {
                /* Clear the IRQ flag */
                ST25R_IRQ_FLAG = 0;
                return ST25R_OK;
            }
            /* Clear the IRQ flag */
            ST25R_IRQ_FLAG = 0;
        }
    }

    return ST25R_IRQ_ERROR;
}

/**
 * @brief Wait for a direct command to be executed
 * 
 * @return byte Return values:
 *         - ST25R_OK if the direct command has been executed
 *         - ST25R_IRQ_ERROR if an error occurs during IRQ handling 
 */
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

static byte CheckForSpecificIRQ(irq_reason_t irqReason) {
    if (ST25R_IRQ_FLAG == 1) {
        if (ST25R_IRQ_REASON == irqReason) {
            return ST25R_OK;
        }

        /* Clear the IRQ flag */
        ST25R_IRQ_FLAG = 0;
    }

    return ST25R_IRQ_ERROR;
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

    /* Send data */
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

    /* Send data */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, txBuf, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }

    /* Receive data */
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
    byte txBuf[FIFO_SIZE + 1] = {0};

    /* Assert parameters */
    if ((pData == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }
    if (len > FIFO_SIZE) {
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
    if ((pData == NULL) || (len == 0)) {
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
    if (fifoLen > FIFO_SIZE) {
        return ST25R_UNKNOWN_ERROR;
    }

    /* Check FIFO status register 2 */
    if (ST25R_ReadRegister(ST25R_FIFO2_STATUS_REG, &regData) != ST25R_OK) {
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

    /* Send read fifo byte command */
    SPI1_RESET_NSS();
    if (HAL_SPI_Transmit(&hspi1, &flag, 1, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    
    /* Read fifo */
    if (HAL_SPI_Receive(&hspi1, pData, (ushort)fifoLen, MAX_SPI_TIMEOUT) != HAL_OK) {
        SPI1_SET_NSS();
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    SPI1_SET_NSS();

    *len = fifoLen;

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
 * @return byte ST25R_IRQ_ERROR if an error occurs during IRQ handling
 * @return byte ST25R_PWR_SUPPLY_ERROR if an error occurs during power supply measurement
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_PowerUpSequence(void) {

    /* Local variables */
    byte regData = 0;

    /* Power-Up Sequence (see section 1.2.16 of DS11793) */

    /* Send "Set Default" direct command */
    if (ST25R_SendDirectCommand(ST25R_SET_DEFAULT_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }

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

    /* Measure and power supply */
    ushort voltage;
    if (ST25R_MeasurePowerSupply(&voltage, ST25R_MEAS_SRC_VDD) != ST25R_OK) {
        return ST25R_PWR_SUPPLY_ERROR;
    }
    if ((voltage < 2400) || (voltage > 5500)) {
        return ST25R_PWR_SUPPLY_ERROR;
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

    /* Configure Mode Definition register (Type A selected by default) */
    ST25R_MODIFY_REG(regData, ST25R_ISO14443A_OPMODE << ST25R_OPMODE_POS, ST25R_OPMODE_MASK);
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
    regData = 16;  // MRT < FDT + 64/fc
    status = ST25R_WriteRegister(ST25R_MASK_RX_TIMER_REG, regData);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure No Response Timer register to be compliant with ISO14443A FDT */
    byte regDataArray[3] = {0, 100, 0}; // NRT > FDT + 64/fc
    status = ST25R_WriteMultipleRegisters(ST25R_NO_RESP_TIMER1_REG, regDataArray, 3);
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
    if (WaitForSpecificIRQ_T(ST25R_OSC_FREQ_STABLE_IRQ, 100) != ST25R_OK) {
        /* Check that bit osc_ok of Auxiliary Display register is set */
        ST25R_ReadRegister(ST25R_AUX_DISPLAY_REG, &regData);
        if (!LBIT4_MASK(regData)) {
            return ST25R_UNKNOWN_ERROR;
        }
    }
    
    /* Calibrate the antenna (optional) */
    if (ST25R_SendDirectCommand(ST25R_CAL_ANTENNA_CMD) != ST25R_OK) {
        return ST25R_SPI_IO_STREAM_ERROR;
    }
    if (WaitForDirectCommand() != ST25R_OK) {
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

    /* RFID protocols usually require that the reader field is turned on 
    for a while before sending the first command (5 ms for ISO14443) */
    HAL_Delay(5);

    return ST25R_OK;

}


/**
 * @brief Select the ISO14443 Type B protocol
 * 
 * @return byte ST25R_OK if the Type B protocol has been selected successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_IRQ_ERROR if an error occurs during IRQ handling
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

    // TODO: Implement the rest of the Type B protocol
    
    return ST25R_OK;

}


/* Measure functions */

/**
 * @brief Measure the power supply of various voltages on the ST25R3911B
 * 
 * @note This function works only if 'en' bit in Operation Control register is set
 * 
 * @param voltage Pointer to the voltage value
 * @param config Configuration of the power supply to be measured
 * 
 * @return byte ST25R_OK if the power supply has been measured successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 * @return byte ST25R_IRQ_ERROR if an error occurs during IRQ handling
 */

byte ST25R_MeasurePowerSupply(ushort *voltage, byte config) {

    /* Assert parameters */
    if (voltage == NULL) {
        return ST25R_WRONG_PARAMETER;
    }
    if (config > ST25R_MEAS_SRC_VSPRF) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Local variables */
    byte regData = 0;
    byte status = ST25R_OK;
    ushort lsb_uv = 23438; // 1 LSB = 23.438 mV = 23438 uV
    word microvolts = 0;

    /* Configure Regulator Voltage Control register */
    status = ST25R_ModifyRegister(ST25R_REGUL_VOLTAGE_CTRL_REG, config, ST25R_MEAS_SRC_MASK);
    if (status != ST25R_OK) {
        return status;
    }

    /* Send direct command for measuring the power */
    status = ST25R_SendDirectCommand(ST25R_MEAS_PWR_SUPPLY_CMD);
    if (status != ST25R_OK) {
        return status;
    }
    status = WaitForDirectCommand();
    if (status != ST25R_OK) {
        return status;
    }

    /* Read out ADC content */
    status = ST25R_ReadRegister(ST25R_AD_CONV_OUT_REG, &regData);
    if (status != ST25R_OK) {
        return status;
    }

    /* Compute the voltage (in mV) */
    microvolts = regData * lsb_uv;
    *voltage = (ushort) (microvolts / 1000);

    return ST25R_OK;
}


/* Generic Tx/Rx functions */

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
byte ST25R_TransmitData(byte *pData, ushort len, byte crcOption) {

    /* Assert parameters */
    if ((pData == NULL) || (len == 0)) {
        return ST25R_WRONG_PARAMETER;
    }
    if (len > MAX_NUMBER_TX_BYTES) {
        return ST25R_WRONG_PARAMETER;
    }
    if (crcOption > TX_CRC_OFF) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Local variables */
    byte status = ST25R_OK;
    //byte regData = 0;
    byte lenArray[2] = {0};

    /* Send 'Clear' direct command to flush the FIFO */
    status = ST25R_SendDirectCommand(ST25R_CLEAR_CMD);
    if (status != ST25R_OK) {
        return status;
    }

    /* Configure the Number of Transmitted Bytes registers 1 and 2 */
    len <<= 3;
    lenArray[0] = MSB(len);
    lenArray[1] = LSB(len);
    status = ST25R_WriteMultipleRegisters(ST25R_NB1_TX_BYTES_REG, lenArray, 2);
    if (status != ST25R_OK) {
        return status;
    }


    /* Case 1. Data length is inferior to FIFO length */
    if (len <= FIFO_SIZE) {

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
        byte nbFifoLoads = len / FIFO_SIZE;
        byte nbLeftBytes = len % FIFO_SIZE;

        for (byte i = 0; i < nbFifoLoads; i++) {

            /* Fill the FIFO */
            status = ST25R_LoadFifo(pData, FIFO_SIZE);
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
            pData += FIFO_SIZE;

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


/**
 * @brief Receive data from the PICC
 * 
 * @note This function is used to receive bytes from the PICC
 * regardless of the selected protocol.
 * 
 * @param pData Pointer to the data buffer
 * @param len Pointer to the length of the data buffer
 * @param timeout Timeout for the reception
 * 
 * @return byte ST25R_OK if the data has been received successfully
 * @return byte ST25R_WRONG_PARAMETER if the parameters are not valid
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 * @return byte ST25R_FIFO_OVFLOW_ERROR if an overflow occurs
 * @return byte ST25R_FIFO_UDFLOW_ERROR if an underflow occurs
 * @return byte ST25R_FIFO_EMPTY_ERROR if the FIFO is empty
 */
byte ST25R_ReceiveData(byte *pData, ushort *len) {

    /* Assert parameters */
    if (pData == NULL) {
        return ST25R_WRONG_PARAMETER;
    }

    /* Clear length parameter */
    *len = 0;

    /* Local variables */
    byte status = ST25R_OK;
    byte nbOfReadBytes = 0;

    /* Case 1. More than FIFO_SIZE bytes to read */
    if (CheckForSpecificIRQ(ST25R_FIFO_WATER_LVL_IRQ) == ST25R_OK) {

        /* Read bytes until End of Reception IRQ */
        do {
            /* Read FIFO */
            status = ST25R_ReadFifo(pData, &nbOfReadBytes);
            if (status != ST25R_OK) {
                return status;
            }

            /* Update parameters */
            pData += nbOfReadBytes;
            *len += nbOfReadBytes;

        } while (CheckForSpecificIRQ(ST25R_RX_STOP_IRQ) != ST25R_OK);

        /* Replace the pointer to its first value */
        pData -= *len;

    }

    /* Case 2. Less than FIFO_SIZE bytes to read */
    else if (WaitForSpecificIRQ(ST25R_RX_STOP_IRQ) == ST25R_OK) {
        status = ST25R_ReadFifo(pData, &nbOfReadBytes);
        if (status != ST25R_OK) {
            return status;
        }

        /* Update length */
        *len = nbOfReadBytes;
    }

    else if (nbOfReadBytes == 0) {
        return ST25R_FIFO_EMPTY_ERROR;
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
    //byte regData = 0;

    /* Set the number of valid bits to 0 */
    status = ST25R_ModifyRegister(ST25R_NB2_TX_BYTES_REG, 0x00, 0x07);
    if (status != ST25R_OK) {
        return status;
    }

    /* Set no_crc_rx bit in Auxiliary Definition register */
    status = ST25R_SetBitInRegister(ST25R_AUX_DEF_REG, 7);
    if (status != ST25R_OK) {
        return status;
    }
    /* Set antcl bit in ISO14443A and NFC 106KBPS Settings register */
    status = ST25R_SetBitInRegister(ST25R_ISO14443A_NFC_REG, 0);
    if (status != ST25R_OK) {
        return status;
    }

    /* Send the REQA command */
    status = ST25R_SendDirectCommand(ST25R_TRANSMIT_REQA_CMD);
    if (status != ST25R_OK) {
        return status;
    }
    /* Wait for the end of transmission */
    if (WaitForSpecificIRQ(ST25R_TX_STOP_IRQ) != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    return status;
}


byte ST25R_TransmitWUPA(void) {
    /* Local variables */
    //byte regData = 0;
    byte status = 0;

    /* Set the number of valid bits to 0 */
    status = ST25R_ModifyRegister(ST25R_NB2_TX_BYTES_REG, 0x00, 0x07);
    if (status != ST25R_OK) {
        return status;
    }

    /* Set no_crc_rx bit in Auxiliary Definition register */
    status = ST25R_SetBitInRegister(ST25R_AUX_DEF_REG, 7);
    if (status != ST25R_OK) {
        return status;
    }
    /* Set antcl bit in ISO14443A and NFC 106KBPS Settings register */
    status = ST25R_SetBitInRegister(ST25R_ISO14443A_NFC_REG, 0);
    if (status != ST25R_OK) {
        return status;
    }

    /* Send WUPA command */
    status = ST25R_SendDirectCommand(ST25R_TRANSMIT_WUPA_CMD);
    if (status != ST25R_OK) {
        return status;
    }
    /* Wait for the end of transmission */
    if (WaitForSpecificIRQ(ST25R_TX_STOP_IRQ) != ST25R_OK) {
        return ST25R_IRQ_ERROR;
    }

    return status;

}


/**
 * @brief Perform the anticollision procedure in ISO14443 Type A mode
 * 
 * @return byte ST25R_OK if the anticollision procedure has been performed successfully
 * @return byte ST25R_SPI_IO_STREAM_ERROR if an error occurs during SPI communication
 * @return byte ST25R_UNKNOWN_ERROR if an unknown error occurs
 */
byte ST25R_PerformAnticollA(void) {
    
    /* Local variables */
    byte status = ST25R_OK;
    byte atqa[2] = {0};
    ushort len = 0;
    byte uidSize = 0;

    /* Polling with WUPA every 2 secs */
    do {

        /* Send WUPA command */
        status = ST25R_TransmitWUPA();
        if (status != ST25R_OK) {
            return status;
        }
        if (WaitForIRQ_T(10) == ST25R_NO_RESP_TIM_IRQ) {
            return ST25R_NO_RESP_ERROR;
        }

        /* Receive ATQA, if there is any */
        status = ST25R_ReceiveData(atqa, &len);
        if (status != ST25R_OK) {
            return status;
        }
        if (CheckForSpecificIRQ(ST25R_NO_RESP_TIM_IRQ) == ST25R_OK) {
            return ST25R_TIMEOUT_ERROR;
        }
        
        /* Wait for 2 seconds */
        HAL_Delay(2000);

    } while ((status != ST25R_OK) && (len != 2));

    /* Analyze the ATQA */
    if (atqa[0] != 0x00) {
        return ST25R_UNKNOWN_ERROR;
    }
    byte uid = atqa[1] & UID_MASK;
    switch (uid) {
        case UID_SINGLE:
            uidSize = UID_SINGLE_SIZE;
            break;
        case UID_DOUBLE:
            uidSize = UID_DOUBLE_SIZE;
            break;
        case UID_TRIPLE:
            uidSize = UID_TRIPLE_SIZE;
            break;
        default:
            return ST25R_UNKNOWN_ERROR;
    }


    /* TODO: Select and anticollision loop */
    /* TODO: Get UID and analyze it */

    return status;
}

/* ISO14443B functions */

