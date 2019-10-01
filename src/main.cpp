#include "Serial.h"

int main()
{
    cli();
    // start communication at a baud rate of 9600
    Serial Serial1(0);
    Serial1.begin(9600);
    sei();

    // read the response data
    while (true) {
        if (Serial1.rxDataAvailable()) {
            uint8_t data = Serial1.read();
            Serial1.write(data);
        }
    }
}
