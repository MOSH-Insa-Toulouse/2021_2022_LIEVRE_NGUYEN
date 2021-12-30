# 2021_2022_LIEVRE_NGUYEN
This project is part of the 5ISS year formation at INSA Toulouse. We created a nanoparticle gas sensor in the AIME laboratory at INSA Toulouse.
Then, we designed the PCB and the code to use the sensor.

## SOMMAIRE 
* 1 [Introduction](#intro)
* 2 [Lora](#p2)
* 3 [LoraWAN](#p3)
* 4 [TTN](#p4)
* 5 [KiCad](#p5) 
* 6 [Node-RED](#p6)
* 7 [Datasheet](#p7)

## Introduction <a name="intro"></a>

This project allowed us to go further in the development of our sensor. 
Indeed, we can now use it with a shield, connect it to TTN and see the data on a Node-RED dashboard.
Here, you will find :

- [x] The code of the sensor to collect data, alert the user and connect it to TTN.
- [x] The PCB shield along with the KiCad project.
- [x] The Node-RED flow for the dashboard.
- [x] The sensor's datasheet.

## Lora <a name="p2"></a>

First, we tested the LoRa connexion with a small code.
To test it, we used a very simple circuit:
- RST connected to D21
- Rx connected to D19
- Tx connected to D18

**Tx :**
To send a message, you need to enter the following command : "radio tx" + hexa_msg
``` c++
#include <Arduino.h>
#include <HardwareSerial.h>
#include "rn2xx3.h"

#define RXD2 18
#define TXD2 19
#define RESET 21

rn2xx3 lora(Serial2);

void initialize_radio()
{
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  Serial2.flush();
}

void setup()
{
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
  if(Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
  if (Serial.available())
  { 
    String Command = Serial.readStringUntil('\n');
    Serial.println(Command);
    Serial2.print(Command + "\r\n");
  }
}
```

**Rx :** 
``` c++
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
}

void setup()
{
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
  Serial2.print("radio rx 0\r\n");
  if(Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
  if (Serial.available())
  { 
    String Command = Serial2.readStringUntil('\n');
    Serial.println(Command);
  }
  delay(1000);
}
```

You can find the codes [here](LINK CODE)

## LoraWAN <a name="p3"></a>

"mac tx cnf 81"+ hexa
Gas sensor v1.5 (MQ2)

## TTN <a name="p4"></a>

To configure your TTN account, you can follow these steps:
1.
2.

## KiCad <a name="p5"></a>

We then proceeded to conceive the PCB shield.
The first thing to do is to draw the circuit. We used the following circuit:
SCREEN CIRCUIT

Then, we needed the footprint of each elements.
SCREEN FOOTPRINT

Finally, we designed the PCB. Here, you have a picture of the schematics and the 3D view:
SCREEN PCB + 3D

## Node-RED <a name="p6"></a>

Finally, we needed a dashboard to display the data andthe controls of the sensor.
Here is a screenshot of the dashboard we created :
SCREEN DASHBOARD

You can find the flow's code [here](LINK FLOW NODE RED)

## Datasheet <a name="p7"></a>

You can find the datasheet [here](LINK DATASHEET)