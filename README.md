# 2021_2022_LIEVRE_NGUYEN
This project is part of the 5ISS year formation at INSA Toulouse. We created a nanoparticle gas sensor in the AIME laboratory at INSA Toulouse.
Then, we designed the PCB and the code to use the sensor and display its data on a dashboard.

## SOMMAIRE 
1. [Introduction](#intro)
2. [Lora](#p2)
3. [LoraWAN](#p3)
4. [TTN](#p4)
5. [KiCad](#p5) 
6. [Node-RED](#p6)
7. [Datasheet](#p7)
8. [Possible improvements](#p8)
9. [Contacts](#p9)

## Introduction <a name="intro"></a>

This project allowed us to go further in the development of our sensor. 
Indeed, we can now use it with a shield, connect it to TTN and see the data on a Node-RED dashboard. The arduino code used to retrieve and process the data was adapted for both the AIME gaz sensor and the industrial grove sensor given to us. We used a ESP32 as a microcontroller.

Here, you will find :

- [x] The arduino code of the sensor to collect data, alert the user and connect it to TTN.
- [x] The PCB shield along with the KiCad project.
- [x] The Node-RED flow for the dashboard.
- [x] The sensor's datasheet.

Here is a picture of our circuit:
![ARDUINO](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/ARDUINO.PNG)

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

You can find the codes:
- [here for Tx](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/tree/main/Piste_verte/Lora_Tx_mod2mod)
- [here for Rx](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/tree/main/Piste_verte/Lora_Rx_mod2mod)

## LoraWAN <a name="p3"></a>

In this part, we implemented the full arduino code that retrieves and sends to TTN via LoraWAN the value of both the grove and AIME sensor. 
We also read the raw value of the resistor (R_alu) that serves as a temperature sensor. By using the R_alu value, we then deduced the temperature.
We also implemented a PID controller to maintain the temperature of the silicium resistor R_Poly to a temperature of 200°C. To do so, we use the PID controller to generate a duty cycle that is then entered as a parameter for the PWM signal that controls the silicium rod.
<img src="/Documents/Pictures/pid_control.png" height="150"><br>
The code corresponding to this part is the following:
``` c++
#include <Arduino.h>
#include "rn2xx3.h"

#define RESET 21 //Pin Reset LoRA
#define RX 18    //Pin RX LoRa
#define TX 19    //Pin TX LoRa
#define INTERRUPT_PIN 12 //Pin d'interruption du capteur de gaz grove
#define PWM_PIN 16   //Pin de sortie de la commande PWM (vers transistor -> R_poly (barreau silicium))
#define GROVE_PIN 4  //Signal d'entree du capteur grove 
#define CMD_R_POLY 7 //Pin de la sortie de commande de la resistance de chauffe (barreau silicium)
#define ADC_SENSOR 33 // Pin d'entree du signal du capteur de gaz (AIME)
#define ADC_R_ALU 27 //Pin d'entree de la resistance d'aluminium (capteur de temperature)
#define BUZZER_PIN 17  //Pin de sortie du buzzer


#define gaz_threshold 1E8 //Valeur seuil pour déclencher l'alarme (buzzer)
 
//Correcteur PID
float e_curr = 0; //erreur à t
float e_prev = 0; //erreur à t-1
int prev_T = 0; //timestamp 1
int curr_T = 0; //timestamp 2
float target_temp = 200; // Température de R_poly voulue


//Paramètres TTN
String AppEUI = "0000000000000000";
String AppKey = "27C93ED04348754075826C46CE00F53D";


//Capteur de température (Alu)
int a; //
int b; // R_aluminium = a * T(Kelvin) + b 

// Pour la commande PWM de la résistance de chauffe
int duty_cycle; //duty cycle du signal PWM pour la commande de la résistance de chauffe
const uint32_t PWM_fq = 20000;
const uint32_t PWMChannel = 0;
const uint32_t resolution = 8;

// Pour le capteur grove
float sensor_grove_volt; //conversion en volt de l'entree du signal du capteur grove
float RS_gas; //valeur de RS dans le gaz
float R0 = 1.7; //résistance du module grove en environnement ambiant 
float ratio; // ratio RS_GAS/RS_air
uint32_t ratio_int;
uint32_t grove_data; //données capteur grove

// Pour le capteur AIME
float sensor_aime_volt; //conversion en volt de l'entree du signal du capteur AIME
uint32_t AIME_data;  //données capteur AIME
int R1 = 100000;
int R2 = 1000;
int R3 = 100000;
int R4;
int R5 = 10000;
int R6; //resistances du circuit de conditionnement (cf circuit Kicad)
int Vcc; // à definir en fonction des grandeurs electriques voulues


rn2xx3 lora(Serial2); //Constructeur de la classe rn2xx3

//Initialisation du module LoRa
void init_Lora(){
  
  //reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  delay(100); //Attente le startup message du module RN2xx3 
  Serial2.flush();
  //Vérification de la communication avec la radio
  String hweui = lora.hweui();
  while (hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = lora.hweui();
  }

  //Retourne le HWEUI pour l'inscrire via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(lora.sysver());
  
  Serial.println("Trying to join TTN");
  bool join_result = false;
  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = lora.initOTAA(AppEUI, AppKey);

  while (!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //Attente d'une minute avant de réessayer
    join_result = lora.init();
  }
  Serial.println("Successfully joined TTN");
}

//Lecture du capteur grove
uint32_t read_grove() 
{
  int sensor_grove_value = analogRead(GROVE_PIN); 
  sensor_grove_volt=(float)sensor_grove_value/4096*5.0; //tension du capteur
  RS_gas = (5.0-sensor_grove_volt)/ sensor_grove_volt; // taux de variation de la résistance selon le gaz
  ratio = (RS_gas/R0);  // ratio = RS/R0
  uint32_t data_sensor = ratio*100;
  return data_sensor;
}

//Lecture du capteur AIME
uint32_t read_AIME(){ 
  
  int sensor_aime_value = analogRead(ADC_SENSOR);
  sensor_aime_volt=(float)sensor_aime_value/4096*5.0;
  float R_sensor=(R1*(R2+R3)*Vcc/(R2*sensor_aime_volt))-R1-R5; 
  return R_sensor;
}

//Lecture de la résistance du capteur de température
uint32_t read_r_alu (){
  int adc_R_alu = analogRead(ADC_R_ALU);
  float adc_R_alu_volt=(float)ADC_R_ALU/4096*3.3;
  float R_Alu=adc_R_alu_volt*R6/(3.3-adc_R_alu_volt);
  return R_Alu;
}

//Calcul de la temperature du capteur avec R aluminium
float read_temperature(){
  
  float temp = a*(read_r_alu()) + b;
  temp = temp - 273; //conversion en C°
  return temp;
}

//Correcteur PID pour maintenir la température du R_poly à 200 °C
int corr_PID(int Kp, int Ki, int Kd){

  int curr_T = micros();
  float delta_t=((float)(curr_T - prev_T))/1E6; 
  e_curr = target_temp - read_temperature(); 
  float e_P = e_curr; //partie proportionnelle
  float e_I = (e_I + e_curr * delta_t); //partie intégrale
  float e_D = (e_curr - e_prev) / (delta_t); //partie dérivatrice
  prev_T = curr_T;
  e_prev = e_curr;
  uint8_t C = Kp * e_P + Ki * e_I+ Kd * e_D;
  if(C > 255) {
    C = 255; //Pour borner la correction entre 0 et 255
  }
  return C; //Duty cycle de la PWM de commande
 
}

//Configuration PWM
void config_PWM(){
  pinMode(PWM_PIN,OUTPUT);
  ledcSetup(PWMChannel, PWM_fq, resolution);
  ledcAttachPin(PWM_PIN, PWMChannel); //Liaison de la channel avec le pin PWM pour le contrôler
}

//Commande de la resistance de chauffe en PWM
void cmd_PWM_Poly(){
  int duty_cycle=corr_PID(1,0,0);
  ledcWrite(PWMChannel,duty_cycle);
}


void setup() {
  Serial.begin(9600);
  config_PWM(); 
  Serial2.begin(57600, SERIAL_8N1, RX, TX);
  init_Lora();
  if (Serial.available()){
    Serial2.print("sys get hwei\r\n"); // recuperation du HWEUI
    Serial.println(Serial2.readStringUntil('\n'));
  }
  digitalWrite(BUZZER_PIN, 0); //inititialisation du buzzer
  
}

void loop() {

  cmd_PWM_Poly();
  grove_data=read_grove();
  AIME_data=read_AIME();
  if(AIME_data > gaz_threshold){

    
    byte payload[4]; //Payload a envoyer sur TTN
    payload[0] = highByte(grove_data);
    payload[1] = lowByte(grove_data);
    payload[2]=highByte(AIME_data);  
    payload[1]=lowByte(AIME_data);
    lora.txBytes(payload,4); 
    
    for (int i=0;i<5; i++){
      digitalWrite(BUZZER_PIN, 1); //active l'alarme à la detection de gaz
      delay(10000);
    }
    digitalWrite(BUZZER_PIN, 0); //inititialisation du buzzer
  
  }
  
}
```


You can find the code [here](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/tree/main/Piste_rouge/Piste_bleue_code)

## TTN <a name="p4"></a>

We configured an application whose overview is shown in the next picture:

![TTN_overview](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/TTN_overview.PNG)

In this app, we added a device:

![TTN_device](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/TTN_device.PNG)
![TTN_device2](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/TTN_device2.PNG)

You can see that we received live data:

![TTN_livedata](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/TTN_livedata.PNG)

We also added a payload formatter:

![TTN_payload_formatter](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/TTN_payload_formatter.PNG)

## KiCad <a name="p5"></a>

We then proceeded to conceive the PCB shield.
The first thing to do is to draw the circuit. We used the following circuit:
![Schematic_PCB](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/Schematic_PCB.PNG)

Then, we needed the footprint of each elements. <br> <br>
- Buzzer

<img src="/Documents/Pictures/S_BUZZER.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_BUZZER.PNG" height="150"> <br>

- Capacitor

<img src="/Documents/Pictures/S_CAPA.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_CAPA.PNG" height="150"> <br>

- Connector 01x08

<img src="/Documents/Pictures/S_CONN.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_CONN.PNG" height="150"> <br>

- Connector Grove 01x04

<img src="/Documents/Pictures/S_CONN_GROVE.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_CONN_GROVE.PNG" height="150"> <br>

- ESP32

<img src="/Documents/Pictures/S_ESP32.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_ESP32.PNG" height="150"> <br>

- Gas Sensor

<img src="/Documents/Pictures/S_GSWO3.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_GSWO3.PNG" height="150"> <br>

- LED

<img src="/Documents/Pictures/S_LED.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_LED.PNG" height="150"> <br>

- LTC

<img src="/Documents/Pictures/S_LTC.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_LTC.PNG" height="150"> <br>

- Resistor

<img src="/Documents/Pictures/S_R.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_R.PNG" height="150" width="600"> <br>

- Switch

<img src="/Documents/Pictures/S_SW.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_SW.PNG" height="150"> <br>

- Transistor

<img src="/Documents/Pictures/S_TRANS.PNG" height="150"> &emsp; <img src="/Documents/Pictures/FP_TRANS.PNG" height="150"> <br>

Finally, we designed the PCB. Here, you have a picture of the schematics and the 3D view:
![PCB](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/PCB.PNG)
![PCB_3D](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/PCB_3D.PNG)
![PCB_3D_FACE](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/PCB_3D_FACE.PNG)
![PCB_3D_DOS](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/PCB_3D_DOS.PNG)

## Node-RED <a name="p6"></a>

Finally, we needed a dashboard to display the data andthe controls of the sensor. For that, we used to code in the LoRa part because since we didn't have the AIME sensor, we could only use the grove sensor.
Here is a screenshot of the dashboard we created :

![NODERED_dashboard_ok](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/NODERED_dashboard_ok.PNG)

It is a simple dashboard that displays the gaz level. In the next picture, you can see the "warning" mode:

![NODERED_dashboard_attention](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/NODERED_dashboard_attention.PNG)

On the left, we also put a "Controls" module to implement a potential control of the sensor and/or the buzzer directly on the dashboard.

Here is a screenshot of the Node-RED flow:

![NODERED_flow](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Pictures/NODERED_flow.PNG)

You can find the flow's code [here](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/4_Piste_noir/flows_AIMEGSWO3.json)

## Datasheet <a name="p7"></a>

You can find the datasheet [here](https://github.com/MOSH-Insa-Toulouse/2021_2022_LIEVRE_NGUYEN/blob/main/Documents/Datasheet_AIME_GSWO3.pdf)

## Possible improvements <a name="p8"></a>

Of course, there are a few improvements that could be done to this project:

1. we are sad not to have been able to really make the PCB board because we put a lot of time and work to create the best PCB board possible. 
2. if we had the PCB board, it would have been fun to test our gas sensor with our real code to see the result of all our work.
3. it would also have been nice to develop our dashboard a little more (for example, implement the "Controls" module to control the sensor and the alarm).
4. the cherry on top would also have been to develop an Android app to display and control our sensor module on our phone.

## Contacts <a name="p9"></a>

**Agathe LIEVRE & Assia NGUYEN**

- 5th year students from INSA Toulouse

- _lievre@insa-toulouse.fr_

- _asnguyen@insa-toulouse.fr_
