# ATMega2560-serial

An interrupt driven Serial communication (USART) Library written in C++ for ATMega 2560

## How to use:
Modify the `CMakeLists.txt` file:

### Set the flash option:
Setting this to `YES` flashes the generated `.hex` file into the micro-controller
```
SET(FLASH YES)
```
### Set the baud rate for flashing:
```
SET(BAUD 115200)
```
### Set the programmer for flashing:
The default Arduino programmer was used in the `CMakeLists.txt`.
```
SET(PROGRAMMER wiring) 
```
### Select the correct port for flashing:
```
set(PORT /dev/ttyACM0)
```

### Building:
Run the following within the extracted folder.
```
mkdir Build
cd Build
cmake ..
make
```
Generated files would be found in the `bin` folder within the parent folder.
If `SET(FLASH ???)` was set as `YES`, as explained earlier, the file is flashed onto the ATMega2560

### Using the library:
Typical usage
* To begin the transmission, call:
  ```
  Serial1.begin();
  ```
	Since the ATMega has 4 UART ports, select either Serial0, Serial1, Serial2 or Serial3.
The begin() function defaults to 9600 baudrate, Asynchronous Normal mode, one stop bit and eight bit frame length.

* To read, use the Read() function:
  ```
  Serial1.Read();
  ```
* To transmit, use the Write() function:
  ```
  Serial1.Write();
  ```
  
###  Functions used:
### Function to configure USART communication
```
void begin(uint32_t baud,
           USARTOperatingMode mode,
           USARTParity parity,
           USARTStopBits stopBits,
           USARTFrameLength frameLength);
```
- `baud` Sets the communication baud rate.Default value is set to 9600  
- `mode` Sets the communication mode. Default value is set to Asynchronous Normal mode - 
`USARTOperatingMode::ASYNC_NORMAL`. The following are  available:  
    `ASYNC_NORMAL        = 0`   Asynchronous Normal mode  
    `ASYNC_DOUBLE_SPEED  = 1`   Asynchronous Double Speed  
    `SYNC_MASTER         = 2`   Synchronous Master mode  
-  `parity` Sets the parity bit to be used. defaulted to none - `USARTParity::NONE`. Possible values are:  
    `NONE    = 0`  
    `EVEN    = 2`  
    `ODD     = 3` 
-  `stopBits` Sets the number of stop bits - default to 1 bit - `USARTStopBits::ONE`. Possible values are:   
values are:  
    `ONE     = 0`  
    `TWO    = 2`  
-  `frameLength` Sets the number of bits per frame - default value of 8 bits - `USARTFrameLength::EIGHT_BITS`. Possible values are:  
    `FIVE_BITS   = 5`  
    `SIX_BITS    = 6`  
    `SEVEN_BITS  = 7`  
    `EIGHT_BITS  = 8`  
    `NINE_BITS   = 9`  
    
### Function to transmit one byte of data
```
void Serial::write(const uint8_t data)
```
-  `data` Data to be transmitted

### Function to transmit a char array. Overloaded write() function
```
void Serial::write(const char *data)
```
-  `data` Pointer to the a char array

### Function reads one byte at a time from the receiver buffer, and increments the tail index by one
```
uint8_t Serial::read()
```
-  `uint8_t` A byte that is returned from the function


