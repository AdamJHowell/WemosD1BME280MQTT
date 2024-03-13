cmake_minimum_required( VERSION 3.27 )
project( WemosD1BME280MQTT )

set( CMAKE_CXX_STANDARD 14 )

include_directories(
    .
    "C:/Users/Adam/AppData/Local/Arduino15/packages/esp8266/hardware/esp8266/3.1.2/cores/esp8266"
    "C:/Users/Adam/AppData/Local/Arduino15/packages/esp8266/hardware/esp8266/3.1.2/libraries/Wire"
    "C:/Users/Adam/AppData/Local/Arduino15/packages/esp8266/hardware/esp8266/3.1.2/libraries/ESP8266WiFi/src"
    "C:/Users/Adam/AppData/Local/Arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/sdk/lwip2/include"
    "C:/Users/Adam/Documents/Arduino/libraries/Adafruit_Unified_Sensor"
    "C:/Users/Adam/Documents/Arduino/libraries/Adafruit_BME280_Library"
)

add_executable(
    WemosD1BME280MQTT
    WemosD1BME280MQTT.ino
    PubSubClient.cpp
    privateInfo.h
)

set_target_properties( WemosD1BME280MQTT PROPERTIES LINKER_LANGUAGE CXX )
