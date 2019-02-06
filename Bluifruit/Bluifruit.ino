/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h" //Bruger den anden fil (config.h)

const int LEDpin = 5;
const int motorpin1 = 6;
const int motorpin2 = 9;
#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
       -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) 
{
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode (LEDpin, OUTPUT); //Sætter LEDpin (som er 5) til output
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  //Tjekker om der er connection til bluefruit 
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) 
    {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();



  ble.verbose(false);  // debug info is a little annoying after this point!


  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  //Give module a new name
  ble.println("AT+GAPDEVNAME=DEERGOD"); // named DEERGOD

  // Check response status
  ble.waitForOK();

  /* Wait for connection */
  while (! ble.isConnected()) 
  {
    delay(500);
  }
  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
int state = 0;
void loop(void)
{
  // Check for user input
  char n, inputs[BUFSIZE + 1];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }
  if (ble.available()) 
  {
    Serial.print("* "); Serial.print(ble.available()); Serial.println(F(" bytes available from BTLE"));
  }  
  // Echo received data
  String message = "";
  while ( ble.available() )
  {
    int c = ble.read();
    Serial.print((char)c);
    //Få knappen til at sende signal
    message.concat((char)c);

    //Serial.print("message: ");
    //Serial.println(message);
    if (message == "1")
    {
      if (state == 0)
      {
        message = "";
        Serial.println("Turning LED ON");
        digitalWrite(LEDpin, HIGH);
        state = 1;
      }
      else
      {
        message = "";
        Serial.println("Turning LED OFF");
        digitalWrite(LEDpin, LOW);
        state = 0;
      }
    }
    else if (message == "2")
    {
      message = "";
      Serial.println("Rolling curtins down");
      digitalWrite(motorpin1, HIGH);
      digitalWrite(moterpin2, LOW);
      stop(10000);
    }
    else if (message == "3")
    {
      Serial.println("Rolling curtins up");
      digitalWrite(motorpin1, LOW);
      digitalWrite(moterpin2, HIGH);
      stop(10000);
    }
    else if (message == "4")
    {
      Serial.println("Stopping curtins");
      digitalWrite(motorpin1, LOW);
      digitalWrite(moterpin2, LOW);
    }
    else
    {
      message == "";  
    }
  }
  delay(1000);
}
