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



#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define READ_INSTRUCTION 


typedef struct Mcp2515 {
    uint16_t buttons;
    uint pinMiso;
    uint pinCs;
    uint pinSck;
    uint pinMosi;
    spi_inst_t * spiPort;
} Mcp2515;

#define READ_BIT 0x80


void mcp2515_init(Mcp2515 mcp2515,
                  uint pinMiso,
                  uint pinCs,
                  uint pinSck,
                  uint pinMosi,
                  spi_inst_t * spiPort) {

}


uint8_t mcp2515_read_status(Mcp2515* mcp2515, uint8_t type)
{
	uint8_t data = 0x00;
	
    gpio_put(mcp2515->pinCs, 0);  // chip select, active low
    spi_read_blocking(mcp2515->spiPort, 0, &data, 1);
    spi_write_blocking(mcp2515->spiPort, &type, 1);
    gpio_put(mcp2515->pinCs, 1);  // chip deselect, active high
	
	
	return data;
}

int main() {

    stdio_init_all();

    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(spi0, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);


    Mcp2515* mcp2515;

    mcp2515_init(*mcp2515, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, spi0);


    while (1) {
        mcp2515_read_status(mcp2515, 0x03);

        sleep_ms(100);
    }

    return 0;
}
