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
  String hweui = lora.hweui();
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
  //join_result = lora.initABP("260B700E", "55EE6FD2D022FD7B3DB52CE465CD60F0", "EE95FA7D5362AA4A050B428B5F9A0BC2");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = lora.initOTAA("0000000000000000", "27C93ED04348754075826C46CE00F53D");

  while (!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = lora.init();
  }
  Serial.println("Successfully joined TTN");
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
  //Serial2.print("mac pause\r\n");
  //Serial.println(Serial2.readStringUntil('\n'));
}

void loop()
{
  //Get data from gas sensor
  float sensor_volt;
  float RS_gas; 
  float R0 = 1.7;  
  float ratio; // Get ratio RS_GAS/RS_air
  uint32_t ratio_int;
  int sensorValue = analogRead(4);
  sensor_volt=(float)sensorValue/4096*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt; // omit * RL

        //-Replace the name "R0" with the value of R0 in the demo of First Test -
  ratio = RS_gas/R0;  // ratio = RS/R0
        //-----------------------------------------------------------------------

  Serial.print("sensor_volt = ");
  Serial.println(sensor_volt);
  Serial.print("RS_ratio = ");
  Serial.println(RS_gas);
  Serial.print("Rs/R0 = ");
  Serial.println(ratio);

  Serial.print("\n\n");

  delay(1000);

  //Encode int as bytes
  ratio_int = (uint32_t) ratio;
  byte payload[2]; //Payload a envoyer sur TTN

  payload[0] = highByte(ratio_int);
  payload[1] = lowByte(ratio_int);

  lora.txBytes(payload, 2);
  
  // put your main code here, to run repeatedly:
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
