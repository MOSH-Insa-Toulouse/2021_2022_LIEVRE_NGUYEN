#include <Arduino.h>
#include "rn2xx3.h"

#define RESET 21 //Pin Reset LoRA
#define RX 18    //Pin RX LoRa
#define TX 19    //Pin TX LoRa
#define INTERRUPT_PIN 12 //Pin d'interruption du capteur de gaz grove
#define PWM_PIN 16   //Pin de sortie de la commande PWM (vers transistor -> R_poly (barreau silicium))
#define GROVE_PIN 4  //Signal d'entree du capteur grove 
#define CMD_R_POLY 16 //Pin de la sortie de commande de la resistance de chauffe (barreau silicium)
#define ADC_SENSOR 33 // Pin d'entree du signal du capteur de gaz (AIME)
#define ADC_R_ALU 27 //Pin d'entree de la resistance d'aluminium (capteur de temperature)
#define BUZZER_PIN 17  //Pin de sortie du buzzer

//Correcteur PID
double Kp;
double Ki;
double Kd;     //Ponderations du PID (à definir en observant la reponse du systeme)
float e_t = 0; //erreur à t
float e_t_1 = 0; //erreur à t-1
int t_1 = 0; //timestamp 1
int t_2 = 0; //timestamp 2
int delta_t=t_2 - t_1; 


//Paramètres TTN
String AppEUI = "0000000000000000";
String AppKey = "27C93ED04348754075826C46CE00F53D";


//Capteur de température (Alu)
int a; //
int b; // R_aluminium = a * T(Kelvin) + b 

// Pour la commande PWM de la résistance de chauffe
int duty_cycle; //duty cycle du signal PWM pour la commande de la résistance de chauffe
const uint32_t PWM_fq = 20000;
const uint32_t ledChannel = 0;
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
int R1, R2, R3, R4, R5, R6; //resistances du circuit de conditionnement (cf circuit Kicad)
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

//Gestion de l'interruption du capteur
void manage_interrupt(){
  digitalWrite(BUZZER_PIN, 1); //active l'alarme à la detection de gaz
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
  float adc_R_alu_volt=(float)ADC_R_ALU/4096*5.0;
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
int corr_PID(){

  int t_2 = micros();
  e_t = 200 - read_temperature();
  float e_P = Kp * e_t;
  float e_I = Ki * e_t * delta_t; 
  float e_D = Kd * (e_t - e_t_1) / (delta_t);
  t_1 = t_2;
  e_t_1 = e_t;
  uint8_t C = e_P + e_D + e_I;
  if(C > 255) {
    C = 255; //Pour borner la correction entre 0 et 255
  }
  return C; //Duty cycle de la PWM de commande
 
}

//Configuration PWM
void config_PWM(){
  pinMode(PWM_PIN,OUTPUT);
  ledcSetup(ledChannel, PWM_fq, resolution);
  ledcAttachPin(PWM_PIN, ledChannel); //Liaison de la channel du Timer avec la pin PWM
}

//Commande de la resistance d'alu de chauffe en PWM
void cmd_PWM_Poly(){
  int duty_cycle=corr_PID();
  ledcWrite(ledChannel,duty_cycle);
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
  attachInterrupt(INTERRUPT_PIN, manage_interrupt, LOW); //Appel de la fonction d'interruption
  
}

void loop() {

  //Encode int as bytes
  byte payload[4]; //Payload a envoyer sur TTN

  grove_data = read_grove();
  AIME_data = read_AIME();
  
  payload[0] = highByte(grove_data);
  payload[1] = lowByte(grove_data);
  payload[2]=highByte(AIME_data);  
  payload[1]=lowByte(AIME_data);
  lora.txBytes(payload, 4); //envoi sur TTN
}
