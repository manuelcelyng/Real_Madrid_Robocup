#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>
#include <stdint.h>

#include "bluetooth.hpp"
#include "dribbleo.hpp"

Dribbleo* dribbleo;

int main() {
    stdio_init_all();
    dribbleo = new Dribbleo();

    initUart(GPIO_UART_TX, GPIO_UART_RX);
    // dribbleo.initDribbleo();

    while(1){
        // dribbleo.activeDribbleo(true, false);
        sleep_ms(2000);
        // dribbleo.activeDribbleo(false, false);
        sleep_ms(1000);

        // gpio_put(1, 1);
        // gpio_put(2, 0);

        // sleep_ms(2000);

        // gpio_put(1, 0);
        // gpio_put(2, 1);

        // sleep_ms(2000);

        // gpio_put(1, false);
        // gpio_put(2, true);

        // sleep_ms(2000);

        

    }

    return 0;
}