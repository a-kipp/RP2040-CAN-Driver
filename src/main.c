
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "mcp2515.c"

#define SPI_PORT spi0
#define CAN_BAUDRATE 500000
#define PIN_MISO 4
#define PIN_CS_A 5
#define PIN_CS_B 3
#define PIN_SCK 6
#define PIN_MOSI 7



int main(){
    stdio_init_all();

    // This example will use SPI0 at 1MHz, wich is the maximum of the MCP2515.
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS_A);
    gpio_set_dir(PIN_CS_A, GPIO_OUT);
    gpio_put(PIN_CS_A, 1);

    gpio_init(PIN_CS_B);
    gpio_set_dir(PIN_CS_B, GPIO_OUT);
    gpio_put(PIN_CS_B, 1);

    Mcp2515 canA;
    mcp2515_init(&canA, PIN_CS_A, CAN_BAUDRATE, SPI_PORT);
    // Controller is in listen-only mode after initialization.
    mcp2515_setOpmode(&canA, LOOPBACK_MODE);
    

    // Speedtest

    CanMessage transmitBuffer = {0};
    transmitBuffer.length = 4;
    transmitBuffer.extendedIdEnabled = false;
    transmitBuffer.standardId = 666;
    transmitBuffer.isRTR = false;

    CanMessage recieveBuffer = {0};

    uint32_t* val_ptr = (uint32_t*)&transmitBuffer.data;
    *val_ptr = 0;

    uint32_t lastVal = 0;
    uint errors = 0;
    uint transmittedMessages = 0;

    absolute_time_t timeStart = get_absolute_time();
    absolute_time_t timeStart1 = get_absolute_time();

    uint32_t lastExtendedID = transmitBuffer.extendedId;

    int64_t delay;

    while (1)
    {
        timeStart1 = get_absolute_time();
        mcp2515_sendMessageBlocking(&canA, &transmitBuffer);
        mcp2515_recieveMessageBlocking(&canA, &recieveBuffer);
        delay = (delay + absolute_time_diff_us(timeStart1, get_absolute_time())) / 2;
        if (*(uint32_t*)transmitBuffer.data != *(uint32_t*)recieveBuffer.data) {
            if (!transmitBuffer.isRTR) errors++;
        }
        if (transmitBuffer.standardId != recieveBuffer.standardId) {
            errors++;
        }
        if (transmitBuffer.length != recieveBuffer.length) {
            errors++;
        }
        if (absolute_time_diff_us(timeStart, get_absolute_time()) > 1000000) {
            timeStart = get_absolute_time();
            printf("%d Kbyte/s    ", (transmittedMessages*8)/1000);
            transmittedMessages = 0;
            printf("Errors %d    ", errors);
            printf("Delay %d us\n", (uint32_t)delay);
        }
        (*(uint32_t*)transmitBuffer.data) ++;
        transmittedMessages++;
    }

    return 0;
}