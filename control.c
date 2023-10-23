#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"


int main() {
    stdio_init_all();

    while (1) {
        printf("Rotacion en Z:  ");
        sleep_ms(1);
    }
}