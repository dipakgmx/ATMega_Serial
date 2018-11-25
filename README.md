# ATMega2560-serial

Serial communacation (USART) on the ATMega 2560 using C++ classes

## How to use:
Modify the `CMakeLists.txt` file:

### Set the flash option:
Setting this to `YES` flashes the generated `.hex` file into the microcontroller
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
### Select the coorect port for flashing:
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
