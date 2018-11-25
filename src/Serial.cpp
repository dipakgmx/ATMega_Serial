//
// Created by dipak on 18.05.18.
//

#include "Serial.h"
#include <stdlib.h>

Serial::Serial(const uint8_t port)
{
    this->portNumber = port;

    switch (portNumber) {

    case 0:
        this->UDRn      = &UDR0;
        this->UCSRnA    = &UCSR0A;
        this->UCSRnB    = &UCSR0B;
        this->UCSRnC    = &UCSR0C;
        this->UBRRnH    = &UBRR0H;
        this->UBRRnL    = &UBRR0L;
        this->TXENn     = TXEN0;
        this->RXENn     = RXEN0;
        this->UDREn     = UDRE0;
        this->U2Xn      = U2X0;
        this->RXCIEn    = RXCIE0;
        this->USBSn     = USBS0;
        this->UPMn0     = UPM00;
        this->UPMn1     = UPM01;
        this->UCSZn0    = UCSZ00;
        this->UCSZn1    = UCSZ01;
        this->UCSZn2    = UCSZ02;
        this->UMSELn0   = UMSEL00;
        this->UMSELn1   = UMSEL01;
        this->UDRIEn    = UDRIE0;
        break;

    case 1:
        this->UDRn      = &UDR1;
        this->UCSRnA    = &UCSR1A;
        this->UCSRnB    = &UCSR1B;
        this->UCSRnC    = &UCSR1C;
        this->UBRRnH    = &UBRR1H;
        this->UBRRnL    = &UBRR1L;
        this->TXENn     = TXEN1;
        this->RXENn     = RXEN1;
        this->UDREn     = UDRE1;
        this->U2Xn      = U2X1;
        this->RXCIEn    = RXCIE1;
        this->USBSn     = USBS1;
        this->UPMn0     = UPM10;
        this->UPMn1     = UPM11;
        this->UCSZn0    = UCSZ10;
        this->UCSZn1    = UCSZ11;
        this->UCSZn2    = UCSZ12;
        this->UMSELn0   = UMSEL10;
        this->UMSELn1   = UMSEL11;
        this->UDRIEn    = UDRIE1;
        break;

    case 2:
        this->UDRn      = &UDR2;
        this->UCSRnA    = &UCSR2A;
        this->UCSRnB    = &UCSR2B;
        this->UCSRnC    = &UCSR2C;
        this->UBRRnH    = &UBRR2H;
        this->UBRRnL    = &UBRR2L;
        this->TXENn     = TXEN2;
        this->RXENn     = RXEN2;
        this->UDREn     = UDRE2;
        this->U2Xn      = U2X2;
        this->RXCIEn    = RXCIE2;
        this->USBSn     = USBS2;
        this->UPMn0     = UPM20;
        this->UPMn1     = UPM21;
        this->UCSZn0    = UCSZ20;
        this->UCSZn1    = UCSZ21;
        this->UCSZn2    = UCSZ22;
        this->UMSELn0   = UMSEL20;
        this->UMSELn1   = UMSEL21;
        this->UDRIEn    = UDRIE2;
        break;

    case 3:
        this->UDRn      = &UDR3;
        this->UCSRnA    = &UCSR3A;
        this->UCSRnB    = &UCSR3B;
        this->UCSRnC    = &UCSR3C;
        this->UBRRnH    = &UBRR3H;
        this->UBRRnL    = &UBRR3L;
        this->TXENn     = TXEN3;
        this->RXENn     = RXEN3;
        this->UDREn     = UDRE3;
        this->U2Xn      = U2X3;
        this->RXCIEn    = RXCIE3;
        this->USBSn     = USBS3;
        this->UPMn0     = UPM30;
        this->UPMn1     = UPM31;
        this->UCSZn0    = UCSZ30;
        this->UCSZn1    = UCSZ31;
        this->UCSZn2    = UCSZ32;
        this->UMSELn0   = UMSEL30;
        this->UMSELn1   = UMSEL31;
        this->UDRIEn    = UDRIE3;
        break;

    default:
        break;

    }
}

/*!
 * Function to initialise USART communication
 * @param baud The USART communication baud rate - defaulted to 9600
 * @param mode Sets the operating mode. Either Asynchronous Normal mode, Asynchronous Double Speed mode or Synchronous
 * Master mode. Possible value are:
 *      ASYNC_NORMAL        = 0,
 *      ASYNC_DOUBLE_SPEED  = 1,
 *      SYNC_MASTER         = 2
 * @param parity Sets the parity bit - defaulted to NONE. Possible values are:
 *      NONE    = 0,
 *      EVEN    = 2,
 *      ODD     = 3
 * @param stopBits Sets the number of stop bits - default to ONE. Possible values are:
 *      ONE = 1,
 *      TWO = 2
 * @param frameLength Sets the number of bits per frame - default value of 8 bits(EIGHT_BITS). Possible values are:
 *      FIVE_BITS   = 5,
 *      SIX_BITS    = 6,
 *      SEVEN_BITS  = 7,
 *      EIGHT_BITS  = 8,
 *      NINE_BITS   = 9
 */
void Serial::begin(const uint32_t baud,
                   const USARTOperatingMode mode,
                   const USARTParity parity,
                   const USARTStopBits stopBits,
                   const USARTFrameLength frameLength)
{
    this->Parity = parity;
    this->setParity(Parity);

    this->StopBits = stopBits;
    this->setStopBit(StopBits);

    this->FrameLength = frameLength;
    this->setFrameLength(FrameLength);

    this->flushBuffer();

    /* Enable receiver and transmitter */
    *(this->UCSRnB) |= (1 << this->TXENn);
    *(this->UCSRnB) |= (1 << this->RXENn);
    *(this->UCSRnB) |= (1 << this->RXCIEn);

    this->OperatingMode = mode;
    this->setOpMode(baud, OperatingMode);
}

/*!
 * Function sets the Sets the operating mode. Either Asynchronous Normal mode, Asynchronous Double Speed mode or
 * Synchronous Master mode
 * @param baud The USART communication baud rate
 * @param mode The desired operating mode
 */
void Serial::setOpMode(const uint32_t baud,
                    const enum USARTOperatingMode mode)
{
    uint16_t bRate = 0;

    // compute baud rate value for UBRR register
    /***********************************************************************/
    /*UMSELn Bits Settings                                                 */
    /*      UMSELn1         UMSELn0         Mode                           */
    /*        0                0            Asynchronous USART             */
    /*        0                1            Synchronous USART              */
    /*        1                0            (Reserved)                     */
    /*        1                1            Master SPI (MSPIM)             */
    /***********************************************************************/
    switch (mode) {

        case USARTOperatingMode::SYNC_MASTER:
            bRate = static_cast<uint16_t>((F_CPU / 2 / baud - 1) / 2);
            *(this->UCSRnC) |= (1 << this->UMSELn0);
            *(this->UCSRnC) &= ~(1 << this->UMSELn1);
            break;

        case USARTOperatingMode::ASYNC_DOUBLE_SPEED:
            bRate = static_cast<uint16_t>((F_CPU / 4 / baud - 1) / 2);
            // UMSELn0 = 0 and UMSELn1 = 0 for asynchronous mode
            *(this->UCSRnC) &= ~(1 << this->UMSELn0);
            *(this->UCSRnC) &= ~(1 << this->UMSELn1);
            break;

        case USARTOperatingMode::ASYNC_NORMAL:
            bRate = static_cast<uint16_t>((F_CPU / 16 / baud) - 1);
            // UMSELn0 = 0 and UMSELn1 = 0 for asynchronous mode
            *(this->UCSRnC) &= ~(1 << this->UMSELn0);
            *(this->UCSRnC) &= ~(1 << this->UMSELn1);
            break;
    }

    *(this->UBRRnH) = (uint8_t) (bRate >> 8);
    *(this->UBRRnL) = (uint8_t) (bRate & 0xFF);
}

/*!
 * Function to set the number of bits per frame
 * @param frameLength The frame length value. Possible values are:
 *      FIVE_BITS   = 5,
 *      SIX_BITS    = 6,
 *      SEVEN_BITS  = 7,
 *      EIGHT_BITS  = 8,
 *      NINE_BITS   = 9
 */
void Serial::setFrameLength(enum USARTFrameLength frameLength)
{
    /************************************************************************/
    /* @method                                                              */
    /* Set UART/USART frame length (5 to 9 bits)                            */
    /* @param frameLength                                                   */
    /*          the frame length - values from USARTFrameLength.xxx       */
    /* UCSZn Bits Settings                                                  */
    /*  UCSZn2       UCSZn1      UCSZn0       Character Size                */
    /*    0            0           0               5-bit                    */
    /*    0            0           1               6-bit                    */
    /*    0            1           0               7-bit                    */
    /*    0            1           1               8-bit                    */
    /*    1            1           1               9-bit                    */
    /************************************************************************/
    switch (frameLength) {
        case USARTFrameLength::FIVE_BITS:
            *(this->UCSRnC) &= ~(1 << this->UCSZn0);
            *(this->UCSRnC) &= ~(1 << this->UCSZn1);
            *(this->UCSRnB) &= ~(1 << this->UCSZn2);
            break;

        case USARTFrameLength::SIX_BITS:
            *(this->UCSRnC) |= (1 << this->UCSZn0);
            *(this->UCSRnC) &= ~(1 << this->UCSZn1);
            *(this->UCSRnB) &= ~(1 << this->UCSZn2);
            break;

        case USARTFrameLength::SEVEN_BITS:
            *(this->UCSRnC) &= ~(1 << this->UCSZn0);
            *(this->UCSRnC) |= (1 << this->UCSZn1);
            *(this->UCSRnB) &= ~(1 << this->UCSZn2);
            break;

        case USARTFrameLength::EIGHT_BITS:
            *(this->UCSRnC) |= (1 << this->UCSZn0);
            *(this->UCSRnC) |= (1 << this->UCSZn1);
            *(this->UCSRnB) &= ~(1 << this->UCSZn2);
            break;

        case USARTFrameLength::NINE_BITS:
            // set data transmission mode: 9-bit (UCSZn2 = 1; UCSZn1 = 1; UCSZn0 = 1;)
            *(this->UCSRnC) |= (1 << this->UCSZn0);
            *(this->UCSRnC) |= (1 << this->UCSZn1);
            *(this->UCSRnB) |= (1 << this->UCSZn2);
            break;

        default: // Defaulting to 8 bits
            *(this->UCSRnC) |= (1 << this->UCSZn0);
            *(this->UCSRnC) |= (1 << this->UCSZn1);
            *(this->UCSRnB) &= ~(1 << this->UCSZn2);
            break;
    }
}

/*!
 * Function sets the number of stop bits to be used in the USART communication
 * @param stopBit Sets the number of stop bits - default to ONE. Possible values are:
 *      ONE = 1,
 *      TWO = 2
 */
void Serial::setStopBit(USARTStopBits stopBit)
{
    /************************************************************************/
    /* @method                                                              */
    /* Set UART/USART transmission stop bit                                 */
    /* @param stopBit                                                       */
    /*          the stop bit - values from StopBits.xxx               */
    /* USBS Bit Settings                                                    */
    /* USBSn = 0 => 1 Stop bit                                              */
    /* USBSn = 1 => 2 Stop bit                                              */
    /************************************************************************/
    switch (stopBit) {
        // one stop bit
        case USARTStopBits::ONE:
            *(this->UCSRnC) &= ~(1 << this->USBSn);
            break;
            // two stop bits
        case USARTStopBits::TWO:
            *(this->UCSRnC) |= (1 << this->USBSn);
            break;
    }
}

/*!
 * Function sets the partity bits to be used for the USART communication
 * @param parity Sets the parity bit - defaulted to NONE. Possible values are:
 *      NONE    = 0,
 *      EVEN    = 2,
 *      ODD     = 3
 */
void Serial::setParity(enum USARTParity parity)
{
    /************************************************************************/
    /* @method                                                              */
    /* Set UART/USART transmission parity                                   */
    /* @param parity                                                        */
    /*          the parity - values from USARTParity.xxx                  */
    /*  UPMn Bits Settings                                                  */
    /*  UPMn1       UPMn0           Parity Mode                             */
    /*    0           0             Disabled                                */
    /*    0           1             Reserved                                */
    /*    1           0             Enabled, Even Parity                    */
    /*    1           1             Enabled, Odd Parity                     */
    /************************************************************************/
    switch (parity) {
        case USARTParity::NONE:
            *(this->UCSRnC) &= ~(1 << this->UPMn0);
            *(this->UCSRnC) &= ~(1 << this->UPMn1);
            break;

        case USARTParity::EVEN:
            *(this->UCSRnC) &= ~(1 << this->UPMn0);
            *(this->UCSRnC) |= (1 << this->UPMn1);
            break;

        case USARTParity::ODD:
            *(this->UCSRnC) |= (1 << this->UPMn0);
            *(this->UCSRnC) |= (1 << this->UPMn1);
            break;
    }
}

/*!
 * Clear the buffers - reset buffer pointers to their initial state
 * @return Either true or False.
 */
bool Serial::clear()
{
    /************************************************************************/
    /* @method                                                              */
    /* Clear the buffers - reset buffer pointers to their initial state     */
    /* @return true so it can be used in logical expressions with ease      */
    /************************************************************************/
    bool txEnabled = (*(this->UCSRnB) & (1 << this->TXENn)) > 0;
    // disable USART data receiving until clearing the buffer
    if (txEnabled) {
        *(this->UCSRnB) &= ~(1 << this->TXENn);
    }
    this->flushBuffer();
    if (txEnabled) {
        *(this->UCSRnB) |= (1 << this->TXENn);
    }
    return true;
}

/*!
 * Function flushess the transmission and receive buffer indices */
 */
void Serial::flushBuffer()
{
    this->rxHeadIndex = 0;
    this->rxTailIndex = 0;
    this->txHeadIndex = 0;
    this->txTailIndex = 0;
}

/*!
 * Function reads one byte at a time from the receiver buffer, and increments the tail index by one
 * @return one byte from the receiver buffer
 */
uint8_t Serial::read()
{
    uint8_t tmptail;
    /* calculate /store buffer index */
    tmptail = static_cast<uint8_t>((this->rxTailIndex + 1) & (BUFFER_SIZE - 1));
    this->rxTailIndex = tmptail;
    /* get data from receive buffer */
    return rxBuffer[tmptail];
}

/*!
 * Function to transmit one byte of data
 * @param data Data to be transmitted
 */
void Serial::write(const uint8_t data)
{
    // Create a temporary index to point to the head of the buffer
    uint8_t tmp_head_index;
    // Calculate the next value of the head index
    tmp_head_index = static_cast<uint8_t>((txHeadIndex + 1) & (BUFFER_SIZE - 1));

    while (tmp_head_index == Serial::txTailIndex) { ;/* wait for free space in buffer */
    }
    // Load data into the transmission buffer and the next free position i.e., tmp_head_index
    txBuffer[tmp_head_index] = data;
    // Update global index to the incremented value
    txHeadIndex = tmp_head_index;
    // Enable UDRE interrupt
    *(this->UCSRnB) |= (1 << this->UDRIEn); // enable UDRE interrupt
}

/*!
 * Function to transmit a char array. Overloaded write() function
 * @param data Pointer to the a char array
 */
void Serial::write(const char *data)
{
    while ((*data) != '\0')   // Looping until end of char array '\0' is encountered
        write(static_cast<const uint8_t>(*data++));
}

/*!
 * Functions lets know if there is any data received on the receiver buffer
 * @return
 */
bool Serial::rxDataAvailable()
{
    return rxHeadIndex != rxTailIndex;
}

/*!
 * Destructor
 */
Serial::~Serial()
{

}

// define the global USART object(s)
// used to read/write via USART/Serial
// and set the USARTn_RX_vect interrupts
#if defined(HAS_USART)
Usart USART( 0);
  ISR( USART_RX_vect) {
    rxVector( UDR0, USART);
  };
#endif


// Rx & UDRE vectors for USART0
#if defined(HAS_USART0)

    Serial Serial0(0);

    ISR(USART0_RX_vect)
    {
        /* Read the received data */
        uint8_t data = UDR0;
        /* Calculate buffer index */
        uint8_t temp_head = static_cast<uint8_t>((Serial0.rxHeadIndex + 1) & (BUFFER_SIZE - 1));
        if (temp_head == Serial0.rxTailIndex) {
            /* ERROR! Receive buffer overflow */
            /* Do nothing for now */
        }
        else {
            Serial0.rxHeadIndex = temp_head;
            Serial0.rxBuffer[Serial0.rxHeadIndex] = data;
        }
    };
    ISR(USART0_UDRE_vect)
    {
        unsigned char temp_tail;
        /* Check if all data is transmitted */
        if (Serial0.txHeadIndex != Serial0.txTailIndex) {
            /* Calculate buffer index */
            temp_tail = static_cast<unsigned char>((Serial0.txTailIndex + 1) & (BUFFER_SIZE - 1));
            /* Store new index */
            Serial0.txTailIndex = temp_tail;
            /* Start transmission */
            UDR0 = Serial0.txBuffer[temp_tail];
        }
        else {
            UCSR0B &= ~(1 << UDRIE0); // disable UDRE interrupt
        }
    };
#endif

// Rx & UDRE vectors for USART1
#if defined(HAS_USART1)

    Serial Serial1(1);

    ISR(USART1_RX_vect)
    {
        /* Read the received data */
        uint8_t data = UDR1;
        /* Calculate buffer index */
        uint8_t temp_head = static_cast<uint8_t>((Serial1.rxHeadIndex + 1) & (BUFFER_SIZE - 1));
        if (temp_head == Serial1.rxTailIndex) {
            /* ERROR! Receive buffer overflow */
            /* Do nothing for now */
        }
        else {
            Serial1.rxHeadIndex = temp_head;
            Serial1.rxBuffer[Serial1.rxHeadIndex] = data;
        }
    };

    ISR(USART1_UDRE_vect)
    {
        uint8_t temp_tail;
        /* Check if all data is transmitted */
        if (Serial1.txHeadIndex != Serial1.txTailIndex) {
            /* Calculate buffer index */
            temp_tail = static_cast<uint8_t>((Serial1.txTailIndex + 1) & (BUFFER_SIZE - 1));
            /* Store new index */
            Serial1.txTailIndex = temp_tail;
            /* Start transmission */
            UDR1 = Serial1.txBuffer[temp_tail];
        }
        else {
            UCSR1B &= ~(1 << UDRIE1); // disable UDRE interrupt
        }
    };
#endif

// Rx & UDRE vectors for USART2
#if defined(HAS_USART2)

    Serial Serial2(2);

    ISR(USART2_RX_vect)
    {
        /* Read the received data */
        uint8_t data = UDR2;
        /* Calculate buffer index */
        uint8_t temp_head = static_cast<uint8_t>((Serial2.rxHeadIndex + 1) & (BUFFER_SIZE - 1));
        if (temp_head == Serial2.rxTailIndex) {
            /* ERROR! Receive buffer overflow */
            /* Do nothing for now */
        }
        else {
            Serial2.rxHeadIndex = temp_head;
            Serial2.rxBuffer[Serial2.rxHeadIndex] = data;
        }
    };

    ISR(USART2_UDRE_vect)
    {
        uint8_t temp_tail;
        /* Check if all data is transmitted */
        if (Serial2.txHeadIndex != Serial2.txTailIndex) {
            /* Calculate buffer index */
            temp_tail = static_cast<uint8_t>((Serial2.txTailIndex + 1) & (BUFFER_SIZE - 1));
            /* Store new index */
            Serial2.txTailIndex = temp_tail;
            /* Start transmission */
            UDR2 = Serial2.txBuffer[temp_tail];
        }
        else {
            UCSR2B &= ~(1 << UDRIE2); // disable UDRE interrupt
        }
    };
#endif

// Rx & UDRE vectors for USART3
#if defined(HAS_USART3)

    Serial Serial3(3);

    ISR(USART3_RX_vect)
    {
        /* Read the received data */
        uint8_t data = UDR3;
        /* Calculate buffer index */
        uint8_t temp_head = static_cast<uint8_t>((Serial3.rxHeadIndex + 1) & (BUFFER_SIZE - 1));
        if (temp_head == Serial3.rxTailIndex) {
            /* ERROR! Receive buffer overflow */
            /* Do nothing for now */
        }
        else {
            Serial3.rxHeadIndex = temp_head;
            Serial3.rxBuffer[Serial3.rxHeadIndex] = data;
        }
    };

    ISR(USART3_UDRE_vect)
    {
        uint8_t temp_tail;
        /* Check if all data is transmitted */
        if (Serial3.txHeadIndex != Serial3.txTailIndex) {
            /* Calculate buffer index */
            temp_tail = static_cast<uint8_t>((Serial3.txTailIndex + 1) & (BUFFER_SIZE - 1));
            /* Store new index */
            Serial3.txTailIndex = temp_tail;
            /* Start transmission */
            UDR3 = Serial3.txBuffer[temp_tail];
        }
        else {
            UCSR3B &= ~(1 << UDRIE3); // disable UDRE interrupt
        }
    };
#endif