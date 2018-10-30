//
// Created by dipak on 18.05.18.
//

#ifndef ATMEGA_SERIAL_H
#define ATMEGA_SERIAL_H


#include <avr/interrupt.h>
#include <stdlib.h>


#if defined(UBRRH)
    #define HAS_USART
#endif

#if defined(UBRR0H)
    #define HAS_USART0
#endif

#if defined(UBRR1H)
    #define HAS_USART1
#endif

#if defined(UBRR2H)
    #define HAS_USART2
#endif

#if defined(UBRR3H)
    #define HAS_USART3
#endif

#ifndef BUFFER_SIZE
    #define BUFFER_SIZE 64
#endif

/****************************************************************/
/* Enumeration of communication modes supported by USART        */
/****************************************************************/
enum class UsartModeEL
{
    ASYNC_NORMAL = 0,
    ASYNC_DOUBLE_SPEED = 1,
    SYNC_MASTER = 2
};

/****************************************************************/
/* Enumeration defining communication parity modes              */
/****************************************************************/
enum class UsartParityEL
{
    NONE = 0,
    EVEN = 2,
    ODD = 3
};

/****************************************************************/
/* Enumeration defining communication stop bits                 */
/****************************************************************/
enum class UsartStopBitEL
{
    ONE = 1,
    TWO = 2
};

/****************************************************************/
/* Enumeration defining communication frame length              */
/****************************************************************/
enum class UsartFrameLengthEL
{
    FIVE_BITS = 5,
    SIX_BITS = 6,
    SEVEN_BITS = 7,
    EIGHT_BITS = 8,
    NINE_BITS = 9
};

class Serial
{
public:
    Serial(uint8_t portNumber);
    ~Serial();
    void setFrameLength(UsartFrameLengthEL frameLength);
    UsartFrameLengthEL getFrameLength();
    void setStopBit(UsartStopBitEL stopBit);
    UsartStopBitEL getStopBit();
    void setParity(UsartParityEL parity);
    UsartParityEL getParity();
    void begin(const uint32_t baud, uint16_t rxBuffMaxLen, UsartModeEL mode);
    inline void begin(const uint32_t baud, UsartModeEL mode = UsartModeEL::ASYNC_NORMAL)
    { begin(baud, 64, mode); };
    bool clear();
    // friend operation which deals with USARTn_RX_vector(s) interrupts
    // using friend operation allows to access private fields of this class
    void flushBuffer();
    uint8_t read();
    void write(const uint8_t data);
    void write(const char *data);
    bool rxDataAvailabe();

    // Buffer Setup
    // Receiving buffer - Rx
    volatile uint8_t rxBuffer[BUFFER_SIZE];
    volatile uint8_t rxHeadIndex;
    volatile uint8_t rxTailIndex;
    // Transmission buffer - Rx
    volatile uint8_t txBuffer[BUFFER_SIZE];
    volatile uint8_t txHeadIndex;
    volatile uint8_t txTailIndex;

    uint8_t portNumber;

    /** MCU used registry */
    volatile uint8_t *UDRn;
    volatile uint8_t *UCSRnA;
    volatile uint8_t *UCSRnB;
    volatile uint8_t *UCSRnC;
    volatile uint8_t *UBRRnH;
    volatile uint8_t *UBRRnL;

    /* MCU dependent registry bits (positions) */
    // TXENn: Transmitter Enable n
    // Writing this bit to one enables the USART Transmitter. The Transmitter will override normal port operation for the
    //TxDn pin when enabled. The disabling of the Transmitter (writing TXENn to zero) will not become effective until
    //ongoing and pending transmissions are completed, that is, when the Transmit Shift Register and Transmit Buffer
    //Register do not contain data to be transmitted. When disabled, the Transmitter will no longer override the TxDn
    //port.
    uint8_t TXENn;
    // UDREn: USART Data Register Empty
    // The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn is one, the buffer is
    //empty, and therefore ready to be written. The UDREn Flag can generate a Data Register Empty interrupt (see
    //description of the UDRIEn bit).
    uint8_t RXENn;
    // UDREn: USART Data Register Empty
    //The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn is one, the buffer is
    //empty, and therefore ready to be written. The UDREn Flag can generate a Data Register Empty interrupt (see
    //description of the UDRIEn bit).
    //UDREn is set after a reset to indicate that the Transmitter is ready.
    uint8_t UDREn;
    // U2Xn: Double the USART Transmission Speed
    //This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.
    //Writing this bit to one will reduce the divisor of the baud rate divider from 16 to 8 effectively doubling the transfer
    //rate for asynchronous communication.
    uint8_t U2Xn;
    // RXCIEn: RX Complete Interrupt Enable n
    //Writing this bit to one enables interrupt on the RXCn Flag. A USART Receive Complete interrupt will be generated
    //only if the RXCIEn bit is written to one, the Global Interrupt Flag in SREG is written to one and the RXCn bit in
    //UCSRnA is set.
    uint8_t RXCIEn;
    //  USBSn: Stop Bit Select
    //This bit selects the number of stop bits to be inserted by the Transmitter. The Receiver ignores this setting.
    uint8_t USBSn;
    // UPMn1:0: Parity Mode
    //These bits enable and set type of parity generation and check. If enabled, the Transmitter will automatically generate
    //and send the parity of the transmitted data bits within each frame. The Receiver will generate a parity value for
    //the incoming data and compare it to the UPMn setting. If a mismatch is detected, the UPEn Flag in UCSRnA will be
    //set.
    uint8_t UPMn0;
    uint8_t UPMn1;
    // UCSZn1:0: Character Size
    //The UCSZn1:0 bits combined with the UCSZn2 bit in UCSRnB sets the number of data bits (Character SiZe) in a
    //frame the Receiver and Transmitter use.
    uint8_t UCSZn0;
    uint8_t UCSZn1;
    uint8_t UCSZn2;
    // UMSELn1:0 USART Mode Select
    //These bits select the mode of operation of the USARTn
    uint8_t UMSELn0;
    uint8_t UMSELn1;
    // UDRIEn: USART Data Register Empty Interrupt Enable n
    //Writing this bit to one enables interrupt on the UDREn Flag. A Data Register Empty interrupt will be generated only
    //if the UDRIEn bit is written to one, the Global Interrupt Flag in SREG is written to one and the UDREn bit in UCSRnA
    //is set.
    uint8_t UDRIEn;
};

#if defined(HAS_USART) || defined(HAS_USART0)
    extern Serial Serial0;
#endif

#if defined(HAS_USART1)
    extern Serial Serial1;
#endif

#if defined(HAS_USART2)
    extern Serial Serial2;
#endif

#if defined(HAS_USART3)
    extern Serial Serial3;
#endif

#endif //ATMEGA_SERIAL_H