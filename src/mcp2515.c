/*
A simple MCP2515 library for the RP2040.
Copyright (C) 2022 Andreas Kipping

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#include <stdio.h>
#include <stdbool.h>
#include <hardware/spi.h>
#include "register.h"

// RTS it not working


typedef struct Mcp2515 {
    uint _pinCs;
    uint _lastMessagePulledFromBuff;
    spi_inst_t* _spiPort;
} Mcp2515;



typedef struct CanMessage {
    uint16_t standardId;
    bool extendedIdEnabled;
    uint32_t extendedId;
    bool isRTR;
    uint8_t length;
    uint8_t data[8];
} CanMessage;



//-------------------------------------------------------------------------------------------------
// Private Begin
//-------------------------------------------------------------------------------------------------


static void mcp2515_reset(Mcp2515 *mcp2515) {
    // MISO  1 1 0 0 0 0 0 0
    // MOSI  _______________
    //      |<-instruction->|

    const uint8_t RESET_INSTRUCTION = 0b11000000;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &RESET_INSTRUCTION, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
    sleep_ms(10);
}



static uint8_t mcp2515_readByte(Mcp2515 *mcp2515, uint8_t address) {
    // MISO  0 0 0 0 0 0 1 1 n n n n n n n n _______________
    // MOSI  _______________ _______________ n n n n n n n n
    //      |<-instruction->|<---address--->|<----data----->|

    const uint8_t BYTE_READ_INSTRUCTION = 0b00000011;
    uint8_t recieveBuffer = 0xAA;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &BYTE_READ_INSTRUCTION, 1);
    spi_write_blocking(mcp2515->_spiPort, &address, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    return recieveBuffer;
}



static void mcp2515_readRxBuffer(Mcp2515 *mcp2515, uint8_t bufferNum, CanMessage* message) {
    // MISO  1 0 0 1 0 n m 0 _______________ ___________
    // MOSI  _______________ n n n n n n n n n n n n n n
    //      |<-instruction->|<----data----->|<----data--...

    uint8_t instruction;
    switch (bufferNum) {
        case 0: instruction = 0b10010000; break;
        case 1: instruction = 0b10010100; break;
        default: printf("buffer doesn't exist");
    }

    uint8_t NO_DATA[8] = {0}; // a can message has 8 bytes of data at max

    uint8_t rxbnsidhRegister = 0;
    uint8_t rxbnsidlRegister = 0;
    uint8_t rxbneid8Register = 0;
    uint8_t rxbneid0Register = 0;
    uint8_t rxbndlcRegister = 0;

    // Consecutive read from recieve registers.
    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &instruction, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &rxbnsidhRegister, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &rxbnsidlRegister, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &rxbneid8Register, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &rxbneid0Register, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &rxbndlcRegister, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, message->data, 8);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    // reading out the message header from registers
    message->standardId = (
        ((uint16_t)rxbnsidhRegister) << 3 |
        ((uint16_t)rxbnsidlRegister) >> 5
    );
    message->extendedIdEnabled = (rxbnsidlRegister & 0b00001000) >> 3;
    if (message->extendedIdEnabled) {
        message->extendedId = (
            (uint32_t)(rxbnsidlRegister & 0b00000011) << 16 |
            (uint32_t)rxbneid8Register << 8 |
            (uint32_t)rxbneid0Register
        );
    } else {
        message->extendedIdEnabled = 0;
    }
    if (message->extendedIdEnabled) {
        if (rxbndlcRegister & 0b01000000) {
            message->isRTR = true;
        } else {
            message->isRTR = false;
        }
    } else {
        if (rxbnsidlRegister & 0b00010000) {
            message->isRTR = true;
        } else {
            message->isRTR = false;
        }
    }
    if (message->isRTR) {
        message->length = 0;
    } else {
        message->length = rxbndlcRegister & 0b00001111;
    }
}



static void mcp2515_writeByte(Mcp2515 *mcp2515, uint8_t address, uint8_t data) {
    // MISO  0 0 0 0 0 0 1 0 n n n n n n n n n n n n n n n n
    // MOSI  _______________ _______________ _______________
    //      |<-instruction->|<---address--->|<----data----->|

    const uint8_t BYTE_WRITE_INSTRUCTION = 0b00000010;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &BYTE_WRITE_INSTRUCTION, 1);
    spi_write_blocking(mcp2515->_spiPort, &address, 1);
    spi_write_blocking(mcp2515->_spiPort, &data, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}



static void mcp2515_loadTxBuffer(Mcp2515 *mcp2515, uint8_t bufferNum, CanMessage* message) {
    // MISO  0 1 0 0 0 a b c n n n n n n n n n n n n n n
    // MOSI  _________ _____ _______________ ___________
    //      |<-instr->|<adr>|<----data----->|<----data--...

    uint8_t instruction;
    switch (bufferNum) {
        case 0: instruction = 0b01000000; break;
        case 1: instruction = 0b01000010; break;
        case 2: instruction = 0b01000100; break;
        default: printf("buffer doesn't exist");
    }

    if (message->isRTR) {
        message->length = 0;
    }

    uint8_t txbnsidhRegister = message->standardId >> 3;
    uint8_t txbnsidlRegister = (
        message->standardId << 5 | 
        (message->extendedIdEnabled & 0b00000001) << 3 |
        message->extendedId >> 16
    );
    uint8_t txbneid8Register = message->extendedId >> 8;
    uint8_t txbneid0Register = message->extendedId;
    uint8_t txbndlcRegister = (
        message->isRTR << 6 |
        message->length & 0b00001111
    );

    // Consecutive write to transmit registers.
    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &instruction, 1);
    spi_write_blocking(mcp2515->_spiPort, &txbnsidhRegister, 1);
    spi_write_blocking(mcp2515->_spiPort, &txbnsidlRegister, 1);
    spi_write_blocking(mcp2515->_spiPort, &txbneid8Register, 1);
    spi_write_blocking(mcp2515->_spiPort, &txbneid0Register, 1);
    spi_write_blocking(mcp2515->_spiPort, &txbndlcRegister, 1);
    spi_write_blocking(mcp2515->_spiPort, message->data, message->length);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}



static void mcp2515_requestToSend(Mcp2515 *mcp2515, uint8_t bufferNum) {
    // MISO  1 0 0 0 0 n n n
    // MOSI  _______________
    //      |<-instruction->|
    //                 | | |
    //                 | | Request-to-Send for TXBO
    //                 | Request-to-Send for TXB1
    //                 Request-to-Send for TXB2

    uint8_t requestToSendInstruction = 0;
    switch (bufferNum) {
        case 0: requestToSendInstruction = 0b10000001; break;
        case 1: requestToSendInstruction = 0b10000010; break;
        case 2: requestToSendInstruction = 0b10000100; break;
        default: printf("buffer does not exist");
    }

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &requestToSendInstruction, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}



static uint8_t mcp2515_readStatus(Mcp2515 *mcp2515) {
    // MISO  1 0 1 0 0 0 0 0 _______________ _______________
    // MOSI  _______________ n n n n n n n n n n n n n n n n
    //      |<-instruction->|<----data----->|<-repeat-data->|
    //                       | | | | | | | |
    //                       | | | | | | | RX0IF (CANINTF[0])
    //                       | | | | | | RX1IF (CANINTF[1])
    //                       | | | | | TXREQ (TXB0CNTRL[3])
    //                       | | | | TX0IF (CANINTF[2])
    //                       | | | TXREQ (TXB1CNTRL[3])
    //                       | | TX1IF (CANINTF[3])
    //                       | TXREQ (TXB2CNTRL[3])
    //                       TX2IF (CANINTF[4]

    const uint8_t READ_STATUS_INSTRUCTION = 0b10100000;
    uint8_t recieveBuffer = 0;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &READ_STATUS_INSTRUCTION, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    return recieveBuffer;
}



static uint8_t mcp2515_rxStatus(Mcp2515 *mcp2515) {
    // MISO  1 0 1 1 0 0 0 0 _______________ _______________
    // MOSI  _______________ n n n n n n n n n n n n n n n n
    //      |<-instruction->|<----data----->|<-repeat-data->|
    //                       | |   | | | | |
    //                       | |   | | Filter Match
    //                       | |   | | 0 0 0 RXF0
    //                       | |   | | 0 0 1 RXF1
    //                       | |   | | 0 1 0 RXF2
    //                       | |   | | 0 1 1 RXF3
    //                       | |   | | 1 0 0 RXF4
    //                       | |   | | 1 0 1 RXF5
    //                       | |   | | 1 1 0 RXF0 (rollover to RXB1)
    //                       | |   | | 1 1 1 RXF1 (rollover to RXB1)
    //                       | |   | |
    //                       | |   Received Message Type
    //                       | |   0 0 Standard data frame
    //                       | |   0 1 Standard remote frame
    //                       | |   1 0 Extended data frame
    //                       | |   1 1 Extended remote frame
    //                       | |
    //                       Received Message
    //                       0 0 No RX message
    //                       0 1 Message in RXB0
    //                       1 0 Message in RXB1
    //                       1 1 Messages in both buffers*

    const uint8_t RX_STATUS_INSTRUCTION = 0b10110000;
    uint8_t recieveBuffer = 0;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &RX_STATUS_INSTRUCTION, 1);
    spi_read_blocking(mcp2515->_spiPort, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    return recieveBuffer;
}



static void mcp2515_bitModify(Mcp2515 *mcp2515, uint8_t address,
                              uint8_t mask, uint8_t data) {
    // MISO  0 0 0 0 0 1 0 1 n n n n n n n n n n n n n n n n n n n n n n n n
    // MOSI  _______________ _______________ _______________ _______________
    //      |<-instruction->|<---address--->|<----mask----->|<----data----->|

    const uint8_t BIT_MODIFY_INSTRUCTION = 0b00000101;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(mcp2515->_spiPort, &BIT_MODIFY_INSTRUCTION, 1);
    spi_write_blocking(mcp2515->_spiPort, &address, 1);
    spi_write_blocking(mcp2515->_spiPort, &mask, 1);
    spi_write_blocking(mcp2515->_spiPort, &data, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}



//-------------------------------------------------------------------------------------------------
// Private End
//-------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------
// Pubic Begin
//-------------------------------------------------------------------------------------------------


#define NORMAL_MODE 0b00000000
#define SLEEP_MODE 0b00100000
#define LOOPBACK_MODE 0b01000000
#define LISTEN_ONLY_MODE 0b01100000
#define CONFIGURATION_MODE 0b10000000

void mcp2515_setOpmode(Mcp2515 *mcp2515, uint8_t desiredOpMode) {
    uint8_t actualOpMode;
    do {
        // wait till controller is in new op mode
        mcp2515_bitModify(mcp2515, CANCTRL_REGISTER, 0b11100000, desiredOpMode);
        actualOpMode = mcp2515_readByte(mcp2515, CANSTAT_REGISTER) & 0b11100000;
    } while (actualOpMode != desiredOpMode);
}



bool mcp2515_init(Mcp2515 *mcp2515, uint pinCs, uint baudrate, spi_inst_t* spiPort) {

    // Assign chip select pin.
    mcp2515->_pinCs = pinCs;

    // Assign SPI port.
    mcp2515->_spiPort = spiPort;

    // Reset the controller. Controller is in configuration mode after reset.
    mcp2515_reset(mcp2515);
    sleep_ms(10);

    // Setup the baudrate control register CNF3..CNF1.
    uint8_t cnf3;
    uint8_t cnf2;
    uint8_t cnf1;
    switch (baudrate) {
        case 10000:   cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xe7; break;
        case 20000:   cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xd3; break;
        case 50000:   cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xc7; break;
        case 100000:  cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xc3; break;
        case 250000:  cnf3 = 0x03; cnf2 = 0xac; cnf3 = 0x81; break;
        case 500000:  cnf3 = 0x03; cnf2 = 0xac; cnf3 = 0x80; break; 
        case 1000000: cnf3 = 0x00; cnf2 = 0xc0; cnf3 = 0x80; break;    
        default: printf("desired baudrate not avialable");
        return false;
    }
    mcp2515_writeByte(mcp2515, CNF3_REGISTER, cnf3);
    mcp2515_writeByte(mcp2515, CNF2_REGISTER, cnf2);
    mcp2515_writeByte(mcp2515, CNF1_REGISTER, cnf1);

    // Setup interrupt control register CANINTE
    mcp2515_writeByte(mcp2515, CANINTE_REGISTER, 0); // no interrupts used currently

    // Deactivate RXnBF Pins (High Impedance State).
    mcp2515_writeByte(mcp2515, BFPCTRL_REGISTER, 0);

    // Controller has seperate pins for triggering a message transmission, not used.
    mcp2515_writeByte(mcp2515, TXRTSCTRL_REGISTER, 0);

    // test if controller is accesable by reading from previously written registers.
    if (cnf1 != mcp2515_readByte(mcp2515, CNF1_REGISTER)) {
        printf("controller is not accesable\n");
        return false;
    };

    // Set controller to normal mode.
    mcp2515_setOpmode(mcp2515, LISTEN_ONLY_MODE);

    return true;
}



// Try to send out a message, if message can not be transmitted return false.
bool mcp2515_TrySendMessage(Mcp2515 *mcp2515, CanMessage* message) {

    // if all buffers were loaded we wait till all buffers are empty again to
    // preserve order of transmissions. The buffer with the highest number is
    // sending out it's data first.
    int8_t bufferNum = -1;
    bool isMessageSendOut = false;
    int8_t status = mcp2515_readStatus(mcp2515);

    bool isBuf2loaded = status & 0b01000000;
    bool isBuf1loaded = status & 0b00010000;
    bool isBuf0loaded = status & 0b00000100;

    if(!isBuf2loaded && !isBuf1loaded && !isBuf0loaded) {
        bufferNum = 2;
    } else if (!isBuf1loaded && !isBuf0loaded) {
        bufferNum = 1;
    } else if (!isBuf0loaded) {
        bufferNum = 0;
    }

    if(bufferNum == -1) {
        return isMessageSendOut = false;
    } else {
        // Load all transmit related buffers at once.
        mcp2515_loadTxBuffer(mcp2515, bufferNum, message);

        // Set Transmit request flag.
        mcp2515_requestToSend(mcp2515, bufferNum);

        return isMessageSendOut = true; 
    }
}



// Send out a message, block till message has been send out.
void mcp2515_sendMessageBlocking(Mcp2515 *mcp2515, CanMessage* message) {

    bool isMessageSendOut = false;

    while (!isMessageSendOut) {
        isMessageSendOut = mcp2515_TrySendMessage(mcp2515, message);
    }
}



// Get Message from recieve buffer. If no message inside, return false.
bool mcp2515_TryRecieveMessage(Mcp2515* mcp2515, CanMessage* message) {

    bool isMessageInBuffer = true;
    
    uint8_t status = mcp2515_rxStatus(mcp2515);

    uint8_t isRxBuf0Full = status & 0b01000000;
    uint8_t isRxBuf1Full = status & 0b10000000;

    if(mcp2515->_lastMessagePulledFromBuff = 1) {
        if(isRxBuf0Full) {
            mcp2515_readRxBuffer(mcp2515, 0, message);
        } else if(isRxBuf1Full) {
            mcp2515_readRxBuffer(mcp2515, 1, message);
        } else {
            isMessageInBuffer = false;
        }
    } else {
        if(isRxBuf1Full) {
            mcp2515_readRxBuffer(mcp2515, 1, message);
        } else if(isRxBuf0Full)  {
            mcp2515_readRxBuffer(mcp2515, 0, message);
        } else {
            isMessageInBuffer = false;
        }
    }

    return isMessageInBuffer;
}



// Get Message from recieve buffer. If no message is inside, wait till a 
// message has been recieved
void mcp2515_recieveMessageBlocking(Mcp2515* mcp2515, CanMessage* message) {
    
    bool isMessageInBuffer = false;

    while (!isMessageInBuffer) {
        isMessageInBuffer = mcp2515_TryRecieveMessage(mcp2515, message);
    }
}


//-------------------------------------------------------------------------------------------------
// Pubic End
//-------------------------------------------------------------------------------------------------
