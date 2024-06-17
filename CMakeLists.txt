cmake_minimum_required(VERSION 3.10)

# Set the project name
project(RC_Car_Project)

# Set the path to your Arduino SDK
set(ARDUINO_SDK_PATH "/path/to/arduino/sdk")

#Need to add arduino dependencies

# Define the board type and port
set(ARDUINO_BOARD "arduino:avr:uno")
set(ARDUINO_PORT "/dev/ttyACM0")
set(UPLOAD_METHOD "STK500")

# Add the Arduino library
add_library(arduino STATIC IMPORTED)
set_target_properties(arduino PROPERTIES IMPORTED_LOCATION ${ARDUINO_SDK_PATH}/lib/arduino.a)

# Add each .ino file as a target
file(GLOB INO_FILES "${CMAKE_SOURCE_DIR}/*.ino")

foreach(INO_FILE ${INO_FILES})
    get_filename_component(TARGET_NAME ${INO_FILE} NAME_WE)
    add_executable(${TARGET_NAME} ${INO_FILE})
    target_link_libraries(${TARGET_NAME} arduino)
    set_target_properties(${TARGET_NAME} PROPERTIES
            ARDUINO_BOARD ${ARDUINO_BOARD}
            ARDUINO_PORT ${ARDUINO_PORT}
            UPLOAD_METHOD ${UPLOAD_METHOD}
    )
    add_custom_target(upload_${TARGET_NAME}
            COMMAND ${CMAKE_COMMAND} -E echo "Uploading ${TARGET_NAME}..."
            COMMAND ${CMAKE_MAKE_PROGRAM} ${TARGET_NAME}
            COMMAND ${CMAKE_MAKE_PROGRAM} ${TARGET_NAME}_upload
            DEPENDS ${TARGET_NAME}
    )
endforeach()
