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
#define PIN_CS_A 3
#define PIN_CS_B 5
#define PIN_SCK  6
#define PIN_MOSI 7


#define SPI_PORT spi0
#define READ_BIT 0x80




//-------------------------------------------------------------------------------------
typedef struct Mcp2515 {
    uint16_t buttons;
    uint pinMiso;
    uint pinCs;
    uint pinSck;
    uint pinMosi;
    spi_inst_t* spiPort;
} Mcp2515;





static uint8_t mcp2515_readStatus(Mcp2515* mcp2515) {
    // get content of status register
    //
    // MISO  1 0 1 0 0 0 0 0 _______________
    // MOSI  _______________ n n n n n n n n
    //      |<-instruction->|<----data----->|

    const uint8_t READ_STATUS_INSTRUCTION = 0b10100000;
    uint8_t recieveBuffer = 0xAA;

    gpio_put(mcp2515->pinCs, 0);  // chip select, active low
    spi_write_blocking(SPI_PORT, &READ_STATUS_INSTRUCTION, 1);
    spi_read_blocking(SPI_PORT, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->pinCs, 1);  // chip deselect, active low
    sleep_ms(10);

    return recieveBuffer;
}


static void mcp2515_writeByte(Mcp2515* mcp2515, uint8_t address, uint8_t data) {
    // write one byte of data to a register
    //
    // MISO  0 0 0 0 0 0 1 0 n n n n n n n n n n n n n n n n
    // MOSI  _______________ _______________ _______________
    //      |<-instruction->|<---address--->|<----data----->|

    const uint8_t BYTE_WRITE_INSTRUCTION = 0b00000010;

    gpio_put(mcp2515->pinCs, 0);  // chip select, active low
    spi_write_blocking(SPI_PORT, &BYTE_WRITE_INSTRUCTION, 1);
    spi_write_blocking(SPI_PORT, &address, 1);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(mcp2515->pinCs, 1);  // chip deselect, active low
    sleep_ms(10);
}


static uint8_t mcp2515_readByte(Mcp2515* mcp2515, uint8_t address) {
    // write one byte of data to a register
    //
    // MISO  0 0 0 0 0 0 1 1 n n n n n n n n _______________
    // MOSI  _______________ _______________ n n n n n n n n
    //      |<-instruction->|<---address--->|<----data----->|

    const uint8_t BYTE_READ_INSTRUCTION = 0b00000011;
    uint8_t recieveBuffer = 0xAA;

    gpio_put(mcp2515->pinCs, 0);  // chip select, active low
    spi_write_blocking(SPI_PORT, &BYTE_READ_INSTRUCTION, 1);
    spi_write_blocking(SPI_PORT, &address, 1);
    spi_read_blocking(SPI_PORT, 0, &recieveBuffer, 1);
    gpio_put(mcp2515->pinCs, 1);  // chip deselect, active low
    sleep_ms(10);

    return recieveBuffer;
}


static void mcp2515_reset(Mcp2515* mcp2515) {
    // get content of status register
    //
    // MISO  1 1 0 0 0 0 0 0 
    // MOSI  _______________ 
    //      |<-instruction->|

    const uint8_t RESET_INSTRUCTION = 0b11000000;

    gpio_put(mcp2515->pinCs, 0);  // chip select, active low
    spi_write_blocking(SPI_PORT, &RESET_INSTRUCTION, 1);
    gpio_put(mcp2515->pinCs, 1);  // chip deselect, active low
    sleep_ms(10);
}



void mcp2515_init(Mcp2515* mcp2515, uint pinCs) {
    mcp2515->pinCs = pinCs;
    mcp2515_reset(mcp2515);
}



static uint8_t mcp2515_sendMessage(Mcp2515* mcp2515, uint8_t data) {

    const uint8_t TXB0SIDH_REGISTER = 0b00110001;
    const uint8_t TXB0SIDL_REGISTER = 0b00110010;
    const uint8_t TXB0DLC_REGISTER = 0b00110101;
    const uint8_t TXB0D0_REGISTER = 0b00110101;
    const uint8_t TXB0CTRL_REGISTER = 0x30;

    mcp2515_writeByte(mcp2515, TXB0SIDH_REGISTER, 0);
    mcp2515_writeByte(mcp2515, TXB0SIDL_REGISTER, 0);
    mcp2515_writeByte(mcp2515, TXB0DLC_REGISTER, 0);
    mcp2515_writeByte(mcp2515, TXB0D0_REGISTER, data);
    mcp2515_writeByte(mcp2515, TXB0CTRL_REGISTER, 0b00001000);
    return mcp2515_readByte(mcp2515, TXB0CTRL_REGISTER);
}


//-------------------------------------------------------------------------------------




int main() {
    stdio_init_all();

    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS_A);
    gpio_set_dir(PIN_CS_A, GPIO_OUT);
    gpio_put(PIN_CS_A, 1);

    gpio_init(PIN_CS_B);
    gpio_set_dir(PIN_CS_B, GPIO_OUT);
    gpio_put(PIN_CS_B, 1);

    Mcp2515 canA;
    mcp2515_init(&canA, PIN_CS_A);
    Mcp2515 canB;
    mcp2515_init(&canA, PIN_CS_B);
    

    while (1) {

//-------------------------------------------------------------------------------------
        uint8_t status = mcp2515_readStatus(&canA);
//-------------------------------------------------------------------------------------

        //printf("%02X\n", status);

        printf("%02X\n", mcp2515_sendMessage(&canA, 0xBB));

        sleep_ms(100);
    }

    return 0;
}
