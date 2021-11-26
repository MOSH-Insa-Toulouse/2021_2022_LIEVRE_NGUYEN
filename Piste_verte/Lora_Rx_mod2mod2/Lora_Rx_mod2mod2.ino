#include <Arduino.h>
#include "rn2xx3.h"

#define RESET 21
#define RXD2 18
#define TXD2 19


 // RX, TX !! labels on relay board is swapped !!
rn2xx3 myLora(Serial2);

void initialize_radio()
{
  //reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  Serial2.flush();

/*
  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());


  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP("02017201", "8D7FFEF938589D95AAD928C2E2E7E48F", "AE17E567AECC8787F749A62F5541D522");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
*/
}

void setup() {
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(9600);
  // Open serial communications and wait for port to open:


  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
    initialize_radio();
  //Serial.println("Serial Txd is on pin: "+String(TX));
  //Serial.println("Serial Rxd is on pin: "+String(RX));
  //Serial2.print("sys reset\r\n");
  myLora.tx("s");


}




void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
  delay(1000);
  //Serial2.print("mac get devaddr\r\n");
  Serial2.print("mac pause\r\n");
  Serial.println(Serial2.readStringUntil('\n'));  
  delay(1000);
  Serial2.print("radio rx 0\r\n");
  String str=Serial2.readStringUntil('\n');
  Serial.println(str);
  

    /*Serial.println("TXing");
        delay(1000);

    myLora.tx("!"); //one byte, blocking function
    delay(1000);*/
}
