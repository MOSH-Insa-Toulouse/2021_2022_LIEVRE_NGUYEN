#include <Arduino.h>
#include <HardwareSerial.h>
#include "rn2xx3.h"

#define RXD2 18
#define TXD2 19
#define RESET 21

rn2xx3 lora(Serial2);

void initialize_radio()
{
  //reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  Serial2.flush();

  //check communication with radio
  /*String hweui = lora.hweui();
  while (hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = lora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(lora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  //join_result = lora.initABP("260BC0E0", "9AB480190B05E8A8186F182D04FE2300", "91077773C17E66C8E99DAA4FE049DD7C");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = lora.initOTAA("0000000000000000", "91AC07C8F74B81137F607A298C8FDAAE");

  while (!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = lora.init();
  }
  Serial.println("Successfully joined TTN");*/
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
  initialize_radio();
  Serial.println(Serial2.readStringUntil('\n'));
  Serial2.print("radio set pwr 14\r\n");
  Serial.println(Serial2.readStringUntil('\n'));
  Serial2.print("mac pause\r\n");
  Serial.println(Serial2.readStringUntil('\n'));
}

void loop()
{
  // put your main code here, to run repeatedly:
  //lora.tx("LOL");
  if(Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
  if (Serial.available())
  { // if data is available on hardware serial port ==> data is coming from PC or notebook
    //SoftSerial.print("$PMTK104*37\r\n");              // write it to the SoftSerial shield*/
    //Serial2.print("radio rx 0\r\n");
    String Command = Serial.readStringUntil('\n');
    Serial.println(Command);
    Serial2.print(Command + "\r\n");
  }
}
