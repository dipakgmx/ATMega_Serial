#include "Serial.h"
#include "util/delay.h"

int main()
{
    cli();
    // start communication at a baud rate of 115200
    Serial0.begin(9600, USARTOperatingMode::ASYNC_NORMAL);
    Serial1.begin(9600);
    sei();
    // read the response data
    while (true) {
        if (Serial1.rxDataAvailable()) {
            uint8_t data = Serial1.read();
            Serial0.write(data);
        }
    }
}