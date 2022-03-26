/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "register.h"

/* Example code to talk to a MPU9250 MEMS accelerometer and gyroscope.
   Ignores the magnetometer, that is left as a exercise for the reader.

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor SPI) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic MPU9250 board, other
   boards may vary.

   GPIO 4 (pin 6) MISO/spi0_rx-> ADO on MPU9250 board
   GPIO 5 (pin 7) Chip select -> NCS on MPU9250 board
   GPIO 6 (pin 9) SCK/spi0_sclk -> SCL on MPU9250 board
   GPIO 7 (pin 10) MOSI/spi0_tx -> SDA on MPU9250 board
   3.3v (pin 36) -> VCC on MPU9250 board
   GND (pin 38)  -> GND on MPU9250 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
   The particular device used here uses the same pins for I2C and SPI, hence the
   using of I2C names
*/

#define PIN_MISO 4
#define PIN_CS_A 5
#define PIN_CS_B 3
#define PIN_SCK 6
#define PIN_MOSI 7


#define SPI_PORT spi0
#define READ_BIT 0x80


typedef struct Mcp2515 {
    uint _pinCs;
    uint _lastMessagePulledFromBuff;
    spi_inst_t *_spiPort;
} Mcp2515;



typedef struct CanFrame {
    uint16_t canId;
    uint8_t isRTS;
    uint8_t length;
    uint8_t data[8];
} CanFrame;




static void mcp2515_reset(Mcp2515 *mcp2515) {
    // MISO  1 1 0 0 0 0 0 0
    // MOSI  _______________
    //      |<-instruction->|

    const uint8_t RESET_INSTRUCTION = 0b11000000;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(SPI_PORT, &RESET_INSTRUCTION, 1);
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
    spi_write_blocking(SPI_PORT, &BYTE_READ_INSTRUCTION, 1);
    spi_write_blocking(SPI_PORT, &address, 1);
    spi_read_blocking(SPI_PORT, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    return recieveBuffer;
}




static void mcp2515_readRxBuffer(Mcp2515 *mcp2515, uint8_t bufferNum, CanFrame* frame) {
    // MISO  1 0 0 1 0 n m 0 _______________ ___________
    // MOSI  _______________ n n n n n n n n n n n n n n
    //      |<-instruction->|<----data----->|<----data--...

    uint8_t instruction;
    switch (bufferNum) {
        case 0: instruction = 0b10010000; break;
        case 1: instruction = 0b10010100; break;
        default: printf("buffer doesn't exist");
    }

    uint8_t rxbnsidhContent;
    uint8_t rxbnsidlContent;
    uint8_t rxbneid8Content; // TODO implement extended Id
    uint8_t rxbneid0Content;   // TODO implement extended Id
    uint8_t rxbndlcContent;

    // Consecutive write to transmit registers.
    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(SPI_PORT, &instruction, 1);
    spi_read_blocking(SPI_PORT, 0, &rxbnsidhContent, 1);
    spi_read_blocking(SPI_PORT, 0, &rxbnsidlContent, 1);
    spi_read_blocking(SPI_PORT, 0, &rxbneid8Content, 1);
    spi_read_blocking(SPI_PORT, 0, &rxbneid0Content, 1);
    spi_read_blocking(SPI_PORT, 0, &rxbndlcContent, 1);
    spi_read_blocking(SPI_PORT, 0, frame->data, 8);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low

    frame->canId = 0;
    frame->canId += (uint16_t)rxbnsidhContent << 4;
    frame->canId += (uint16_t)rxbnsidlContent >> 4;
    
    frame->isRTS = (rxbndlcContent & 0b01000000) >> 6;

    frame->length = rxbndlcContent & 0b00001111;
}




static void mcp2515_writeByte(Mcp2515 *mcp2515, uint8_t address, uint8_t data) {
    // MISO  0 0 0 0 0 0 1 0 n n n n n n n n n n n n n n n n
    // MOSI  _______________ _______________ _______________
    //      |<-instruction->|<---address--->|<----data----->|

    const uint8_t BYTE_WRITE_INSTRUCTION = 0b00000010;

    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(SPI_PORT, &BYTE_WRITE_INSTRUCTION, 1);
    spi_write_blocking(SPI_PORT, &address, 1);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}




static void mcp2515_loadTxBuffer(Mcp2515 *mcp2515, uint8_t bufferNum, CanFrame* frame) {
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
    uint8_t txbnsidhContent = frame->canId >> 4;
    uint8_t txbnsidlContent = frame->canId << 4;
    uint8_t rxbneid8Content = 254; // TODO implement extended Id
    uint8_t rxbneid0Content = 1;   // TODO implement extended Id
    uint8_t txbndlcContent = (frame->isRTS << 6) | (frame->length & 0b00001111);

    // Consecutive write to transmit registers.
    gpio_put(mcp2515->_pinCs, 0); // chip select, active low
    spi_write_blocking(SPI_PORT, &instruction, 1);
    spi_write_blocking(SPI_PORT, &txbnsidhContent, 1);
    spi_write_blocking(SPI_PORT, &txbnsidlContent, 1);
    spi_write_blocking(SPI_PORT, &rxbneid8Content, 1);
    spi_write_blocking(SPI_PORT, &rxbneid0Content, 1);
    spi_write_blocking(SPI_PORT, &txbndlcContent, 1);
    spi_write_blocking(SPI_PORT, frame->data, frame->length);
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
    spi_write_blocking(SPI_PORT, &requestToSendInstruction, 1);
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
    spi_write_blocking(SPI_PORT, &READ_STATUS_INSTRUCTION, 1);
    spi_read_blocking(SPI_PORT, 0, &recieveBuffer, 1);
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
    spi_write_blocking(SPI_PORT, &RX_STATUS_INSTRUCTION, 1);
    spi_read_blocking(SPI_PORT, 0, &recieveBuffer, 1);
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
    spi_write_blocking(SPI_PORT, &BIT_MODIFY_INSTRUCTION, 1);
    spi_write_blocking(SPI_PORT, &address, 1);
    spi_write_blocking(SPI_PORT, &mask, 1);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(mcp2515->_pinCs, 1); // chip deselect, active low
}




void mcp2515_init(Mcp2515 *mcp2515, uint pinCs, uint baudrate) {

    // Assign chip select pin.
    mcp2515->_pinCs = pinCs;

    // Reset the controller. Controller is in configuration mode after reset.
    mcp2515_reset(mcp2515);
    sleep_ms(10);

    // Setup the baudrate control register CNF3..CNF1.
    uint8_t cnf3;
    uint8_t cnf2;
    uint8_t cnf1;
    switch (baudrate) {
        case 10:  cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xe7; break;
        case 20:  cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xd3; break;
        case 50:  cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xc7; break;
        case 100: cnf3 = 0x04; cnf2 = 0xb6; cnf3 = 0xc3; break;
        case 250: cnf3 = 0x03; cnf2 = 0xac; cnf3 = 0x81; break;
        case 500: cnf3 = 0x03; cnf2 = 0xac; cnf3 = 0x80; break;    
    default: printf("desired baudrate not avialable");
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

    // Set controller to normal mode and activate clockout
    mcp2515_writeByte(mcp2515, CANCTRL_REGISTER, 0b00000100);

    // test if controller is accesable by reading from previously written registers.
    if (cnf1 != mcp2515_readByte(mcp2515, CNF1_REGISTER))
    {
        printf("controller is not accesable\n");
    };
}




#define NORMAL_MODE 0b00000000
#define SLEEP_MODE 0b00100000
#define LOOPBACK_MODE 0b01000000
#define LISTEN_ONLY_MODE 0b01100000
#define CONFIGURATION_MODE 0b10000000

void mcp2515_setOpmode(Mcp2515 *mcp2515, uint8_t opmode)
{
    mcp2515_bitModify(mcp2515, CANSTAT_REGISTER, 0b11100000, opmode);
}




bool mcp2515_sendMessage(Mcp2515 *mcp2515, CanFrame* frame) {

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
        mcp2515_loadTxBuffer(mcp2515, bufferNum, frame);

        // Set Transmit request flag.
        mcp2515_requestToSend(mcp2515, bufferNum);

        return isMessageSendOut = true; 
    }
}



void mcp2515_sendMessageBlocking(Mcp2515 *mcp2515, CanFrame* frame) {

    bool isMessageSendOut = false;

    while (!isMessageSendOut) {
        isMessageSendOut = mcp2515_sendMessage(mcp2515, frame);
    }
}



bool mcp2515_recieveMessage(Mcp2515* mcp2515, CanFrame* frame) {

    bool isMessageInBuffer = true;
    
    uint8_t status = mcp2515_rxStatus(mcp2515);

    uint8_t isRxBuf0Full = status & 0b01000000;
    uint8_t isRxBuf1Full = status & 0b10000000;

    if(mcp2515->_lastMessagePulledFromBuff = 1) {
        if(isRxBuf0Full) {
            mcp2515_readRxBuffer(mcp2515, 0, frame);
        } else if(isRxBuf1Full) {
            mcp2515_readRxBuffer(mcp2515, 1, frame);
        } else {
            isMessageInBuffer = false;
        }
    } else {
        if(isRxBuf1Full) {
            mcp2515_readRxBuffer(mcp2515, 1, frame);
        } else if(isRxBuf0Full)  {
            mcp2515_readRxBuffer(mcp2515, 0, frame);
        } else {
            isMessageInBuffer = false;
        }
    }

    return isMessageInBuffer;
}



void mcp2515_recieveMessageBlocking(Mcp2515* mcp2515, CanFrame* frame) {
    
    bool isMessageInBuffer = false;

    while (!isMessageInBuffer) {
        isMessageInBuffer = mcp2515_recieveMessage(mcp2515, frame);
    }
}

//-------------------------------------------------------------------------------------

int main(){
    stdio_init_all();

    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 2MHz.
    spi_init(SPI_PORT, 2000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS_A);
    gpio_set_dir(PIN_CS_A, GPIO_OUT);
    gpio_put(PIN_CS_A, 1);

    gpio_init(PIN_CS_B);
    gpio_set_dir(PIN_CS_B, GPIO_OUT);
    gpio_put(PIN_CS_B, 1);

    Mcp2515 canA;
    mcp2515_init(&canA, PIN_CS_A, 500);
    Mcp2515 canB;
    mcp2515_init(&canB, PIN_CS_B, 500);

    CanFrame transmitFrameBuffer = {0};
    CanFrame recieveFrameBuffer = {0};

    CanFrame buffer = {0};
    buffer.length = 8;

    uint32_t* data_ptr = (uint32_t*)&buffer.data;
    *data_ptr = 0;

    uint32_t lastVal = 0;

    while (1)
    {
        *data_ptr = *data_ptr + 1;
        mcp2515_sendMessageBlocking(&canA, &buffer);
        mcp2515_recieveMessageBlocking(&canB, &buffer);
        if (lastVal + 1 != *data_ptr) {
            printf("fail\n");
        }
        lastVal = *data_ptr;
        if (*data_ptr%1000 == 0) {
            printf("%d\n", *data_ptr);
        }
    }

    return 0;
}