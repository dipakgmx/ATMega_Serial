#include "Serial.h"

int main()
{
    cli();
    // start communication at a baud rate of 9600
    Serial serial0(0);
    serial0.begin(9600);
    sei();

    // read the response data
    while (true) {
        if (serial0.rxDataAvailable()) {
            uint8_t data = serial0.read();
            serial0.write(data);
        }
    }
}