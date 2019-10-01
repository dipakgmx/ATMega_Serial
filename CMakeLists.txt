cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_VERBOSE_MAKEFILE TRUE)
project(ATMega_Serial)
#============================================================================================
SET(DEVICE      atmega2560)     #Microcontroller model
SET(FREQ        16000000)       #Frequency
SET(FLASH       NO)            #Turn on/off flashing
SET(BAUD        115200)         #Set flashing baud rate
SET(PROGRAMMER  wiring)         #Programmer
set(PORT        /dev/ttyACM0)   #Insert port number
SET(AVRDUDE     /usr/share/arduino/hardware/tools/avrdude)
SET(AVRCONF     /usr/share/arduino/hardware/tools/avrdude.conf)
SET(ATMELICE    OFF)            #Turn on/off AtmelICE programmer
#============================================================================================
SET(CMAKE_SYSTEM_NAME   Generic)
SET(CMAKE_C_COMPILER    avr-gcc)
SET(CMAKE_CXX_COMPILER  avr-g++)

SET(CSTANDARD	"-std=gnu99")
SET(CPSTANDARD	"-std=gnu++11")
SET(CDEBUG      "-gstabs -g -ggdb")
SET(CWARN       "-Wall -Wstrict-prototypes")
SET(CTUNING     "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT        "-Os")
SET(CMCU        "-mmcu=${DEVICE}")
SET(CDEFS       "-DF_CPU=${FREQ}")


SET(CFLAGS      "${CMCU} ${CDEBUG} ${CDEFS}  ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
SET(CXXFLAGS    "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${CPSTANDARD} ${COPT}")

SET(CMAKE_C_FLAGS                   ${CFLAGS})
SET(CMAKE_CXX_FLAGS                 ${CXXFLAGS})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#============================================================================================

# Create a sources variable with a link to all cpp files to compile
set(SOURCES
        src/main.cpp
        src/Serial.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

#============================================================================================

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-objcopy -O ihex -R.eeprom ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.eep)

ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avr-size ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME} --mcu=${DEVICE} --format=avr)

if(FLASH)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${AVRDUDE} -c{AVRCONF}-v -v -v -v -p${DEVICE} -c${PROGRAMMER} -P${PORT} -b${BAUD} -D -V -U flash:w:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)
endif(FLASH)

if(ATMELICE)
    ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND avrdude -c atmelice_isp -p atmega2560 -U flash:w:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex)
endif(ATMELICE)