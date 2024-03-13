#include "PubSubClient.h"
#include "privateInfo.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h> // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <Wire.h>


Adafruit_BME280 bme280;
WiFiClient wifiClient;
PubSubClient mqttClient( wifiClient );

const float seaLevelPressureHpa     = 1013.25;
const unsigned int printInterval    = 10000; // How long to wait between stat printouts.
unsigned long lastPrintTime         = 0;     // The last time a MQTT publish was performed.
unsigned long lastBrokerConnect     = 0;     // The last time a MQTT broker connection was attempted.
unsigned long brokerCoolDown        = 7000;  // How long to wait between MQTT broker connection attempts.
unsigned long wifiConnectionTimeout = 15000; // The amount of time to wait for a Wi-Fi connection.
char ipAddress[16];                          // A character array to hold the IP address.
char macAddress[18];                         // A character array to hold the MAC address, and append a dash and 3 numbers.
char rssi;                                   // A global to hold the Received Signal Strength Indicator.
unsigned int ledBlinkInterval  = 200;        // Time between blinks.
unsigned long printCount       = 0;          // A counter of how many times the stats have been published.
unsigned long lastLedBlinkTime = 0;          // The last time LED was blinked.
const unsigned int ONBOARD_LED = 2;          // The GPIO which the onboard LED is connected to.
const uint16_t port            = 1883;       // The broker port.

/**
 * @brief readTelemetry() will read the telemetry and save values to global variables.
 */
void readTelemetry()
{
   rssi = WiFi.RSSI();
} // End of readTelemetry() function.

/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
   switch( code )
   {
      case 0:
         snprintf( buffer, 26, "%s", "Idle" );
         break;
      case 1:
         snprintf( buffer, 26, "%s", "No SSID" );
         break;
      case 2:
         snprintf( buffer, 26, "%s", "Scan completed" );
         break;
      case 3:
         snprintf( buffer, 26, "%s", "Connected" );
         break;
      case 4:
         snprintf( buffer, 26, "%s", "Connection failed" );
         break;
      case 5:
         snprintf( buffer, 26, "%s", "Connection lost" );
         break;
      case 6:
         snprintf( buffer, 26, "%s", "Disconnected" );
         break;
      default:
         snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
   }
} // End of lookupWifiCode() function.


/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
   switch( code )
   {
      case -4:
         snprintf( buffer, 29, "%s", "Connection timeout" );
         break;
      case -3:
         snprintf( buffer, 29, "%s", "Connection lost" );
         break;
      case -2:
         snprintf( buffer, 29, "%s", "Connect failed" );
         break;
      case -1:
         snprintf( buffer, 29, "%s", "Disconnected" );
         break;
      case 0:
         snprintf( buffer, 29, "%s", "Connected" );
         break;
      case 1:
         snprintf( buffer, 29, "%s", "Bad protocol" );
         break;
      case 2:
         snprintf( buffer, 29, "%s", "Bad client ID" );
         break;
      case 3:
         snprintf( buffer, 29, "%s", "Unavailable" );
         break;
      case 4:
         snprintf( buffer, 29, "%s", "Bad credentials" );
         break;
      case 5:
         snprintf( buffer, 29, "%s", "Unauthorized" );
         break;
      default:
         snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
   }
} // End of lookupMQTTCode() function.

void printValues()
{
   Serial.println();
   printCount++;

   Serial.printf( "Publish count %ld\n", printCount );
   Serial.println( __FILE__ );
   Serial.println();

   Serial.println( "Wi-Fi info:" );
   Serial.printf( "  MAC address: %s\n", macAddress );
   int wifiStatusCode = WiFi.status();
   char buffer[29]    = "";
   lookupWifiCode( wifiStatusCode, buffer );
   Serial.printf( "  Wi-Fi status text: %s\n", buffer );
   Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
   if( wifiStatusCode == 3 )
   {
      Serial.printf( "  IP address: %s\n", ipAddress );
      Serial.printf( "  RSSI: %hhd\n", rssi );
   }
   Serial.println();

   Serial.println( "MQTT info:" );
   Serial.printf( "  Broker: %s:%d\n", BROKER_IP, port );
   int mqttStateCode = mqttClient.state();
   lookupMQTTCode( mqttStateCode, buffer );
   Serial.printf( "  MQTT state: %s\n", buffer );
   if( mqttClient.connected() )
   {
      Serial.print( "  MQTT broker domain: " );
      Serial.println( mqttClient.getServerDomain() );
      Serial.print( "  MQTT broker IP: " );
      Serial.println( mqttClient.getServerIP() );
      Serial.print( "  MQTT broker port: " );
      Serial.println( mqttClient.getServerPort() );
   }
   float tempC          = bme280.readTemperature();
   float tempF          = ( tempC * 1.8F ) + 32;
   float pressureHpa    = bme280.readPressure() / 100.0F;
   float altitudeMeters = bme280.readAltitude( seaLevelPressureHpa );
   float humidity       = bme280.readHumidity();

   Serial.printf( "Temperature: %.2f °C\n", tempC );

   Serial.printf( "Temperature: %.2f °F\n", tempF );

   Serial.printf( "Pressure: %.2f hPa\n", pressureHpa );

   Serial.printf( "Humidity: %.2f %%\n", humidity );

   Serial.printf( "Altitude: %.1f m\n", altitudeMeters );

   Serial.println();
}

/**
 * @brief wifiBasicConnect() will connect to a SSID.
 */
void wifiBasicConnect()
{
   // Turn the LED off to show Wi-Fi is not connected.
   digitalWrite( ONBOARD_LED, LOW );

   Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", WIFI_SSID );
   WiFi.mode( WIFI_STA );
   WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
   WiFi.setHostname( HOSTNAME );
   WiFi.begin( WIFI_SSID, WIFI_PASSWORD );

   unsigned long wifiConnectionStartTime = millis();

   // Loop until connected, or until wifiConnectionTimeout.
   while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
   {
      Serial.print( "." );
      delay( 1000 );
   }
   Serial.println( "" );

   if( WiFi.status() == WL_CONNECTED )
   {
      // Print that Wi-Fi has connected.
      Serial.println( "\nWi-Fi connection established!" );
      snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      // Turn the LED on to show that Wi-Fi is connected.
      digitalWrite( ONBOARD_LED, HIGH );
      return;
   }
   else
      Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
} // End of wifiBasicConnect() function.

/**
 * @brief mqttConnect() will connect to the MQTT broker.
 */
void mqttConnect()
{
   // Connect the first time.  Avoid subtraction overflow.  Connect after cool down.
   if( lastBrokerConnect == 0 || ( millis() - lastBrokerConnect ) > brokerCoolDown )
   {
      lastBrokerConnect = millis();
      digitalWrite( ONBOARD_LED, LOW );
      Serial.printf( "Connecting to broker at %s:%d...\n", BROKER_IP, port );
      mqttClient.setServer( BROKER_IP, port );

      if( mqttClient.connect( macAddress ) )
         Serial.print( "Connected to MQTT Broker.\n" );
      else
      {
         int mqttStateCode = mqttClient.state();
         char buffer[29];
         lookupMQTTCode( mqttStateCode, buffer );
         Serial.printf( "  MQTT state: %s\n", buffer );
         Serial.printf( "  MQTT state code: %d\n", mqttStateCode );
         return;
      }

      mqttClient.subscribe( "led1" );
      digitalWrite( ONBOARD_LED, HIGH );
   }
} // End of mqttConnect() function.

[[noreturn]] void infiniteLoop()
{
   while( true )
      delay( 10 );
}

void setup()
{
   // Give time to connect to serial after a flash.
   delay( 1000 );
   Serial.begin( 115200 );
   // Pause one second if the serial port is not yet ready.
   if( !Serial )
      delay( 1000 );
   Serial.println();
   Serial.println( "BME280 test" );

   unsigned status = bme280.begin( 0x76 );
   if( !status )
   {
      Serial.println( "Could not find a valid BME280 sensor, check wiring, address, sensor ID!" );
      Serial.print( "SensorID was: 0x" );
      Serial.println( bme280.sensorID(), 16 );
      Serial.print( "        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n" );
      Serial.print( "   ID of 0x56-0x58 represents a BMP 280,\n" );
      Serial.print( "        ID of 0x60 represents a BME 280.\n" );
      Serial.print( "        ID of 0x61 represents a BME 680.\n" );
      Serial.println( "\n" );
      Serial.println( "Going into an infinite delay loop!" );
      infiniteLoop();
   }
   Serial.println();
}

void loop()
{
   if( lastPrintTime == 0 || ( ( millis() - lastPrintTime ) > printInterval ) )
   {
      printValues();
      lastPrintTime = millis();
   }
}
