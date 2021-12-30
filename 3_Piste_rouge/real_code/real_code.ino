#include <Arduino.h>
#include "rn2xx3.h"

#define RST 21 //Pin de reset du module LORA
#define RX 18  //Pin RX du module LORA
#define TX 19  //Pin TX du module LORA
#define INTERRUPT_PIN 12 //Pin d'interruption du capteur de GAZ
#define SIG_grove 2  //Signal d'entree du capteur grove (industriel)
#define Buzz_pin 17  //Pin de sortie du buzzer
#define R_Poly_CMD 16 //Pin de la sortie de commande de la resistance de chauffe (Polysilicium)
#define ADC_SENS 33 // Pin d'entree du signal du capteur de gaz (AIME)
#define ADC_R_Alu 27 //Pin d'entree de la resistance d'ALUMINIUM (capteur de temperature)

int a; //
int b; // R_aluminium = a * T(Kelvin) + b 
String AppEUI = "44E53245F1200CDEEC2525661C76BA38"; //AppEUI du reseau sur TTN
String AppKey = "0000000000000000"; //AppKey du reseau TTN
rn2xx3 myLora(Serial2); //Constructeur de la classe rn2xx3
byte payload[4]; //Payload a envoyer sur TTN
float R0=0.45; //Resistance du module grove en environement ambiant
float sensor_grove_volt; //conversion en volt de l'entree du signal du capteur grove
float sensor_aime_volt; //conversion en volt de l'entree du signal du capteur AIME
float RS_gas; //recoit la valeur de RS dans le gaz
float ratio; // Get ratio RS_GAS/RS_air
int duty_cyle; //duty cycle du signal PWM pour la commande de la resistance de chauffe
int t_1 = 0; //timestamp 1
int t_2 = 0; //timestamp 2
int delta_t=t_2 - t_1; // delta time
double Kp, Ki, Kd; //Ponderations du PID (à definir en observant la reponse du systeme)
float e_t = 0; //mesure de l'erreur a t
float e_t_1 = 0; //mesure de l'erreur a t-1
uint32_t grove_data; //donnée capteur grove
uint32_t aime_data;  //donnée capteur AIME
int R1; //
int R2; //
int R3; //
int R5;  //Resistances du circuit de conditionnement selon la meme numerotation que dans KICAD !
int Vcc; // A definir en fonction des grandeurs electriques voulues
int R6; //
const uint32_t PWM_OUT = 16;  //Pin de sortie de la commande PWM de la resistance de chauffe
// setup des propriétés PWM
const uint32_t freq = 20000;
const uint32_t ledChannel = 0;
const uint32_t resolution = 8;

void initialize_radio() //Initialization du module LORA
{
  //Reset du module
  pinMode(RST, OUTPUT);
  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST, HIGH);
  delay(100); //Attente du message de start du module LORA
  Serial2.flush();
  //check de la communication avec la radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }
  //Recuperation de HWEUI pour la configurer sur TTN
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());
  //message de debug de type "en attente de connexion"
  Serial.println("Trying to join TTN");
  bool join_result = false;
  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = myLora.initOTAA(AppEUI, AppKey);
  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay d'une minute avant retry 
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
  }

void manage_interrupt() //gestion de l'interruption du capteur
{
  digitalWrite(Buzz_pin, 1); //active l'alarme a la detection du GAZ (interruption materielle)
}

uint32_t read_grove_sensor () //Recuperation de la donnée du capteur grove
{
  int sensor_groove_Value = analogRead(SIG_grove); 
  sensor_grove_volt=(float)sensor_groove_Value/4096*5.0; //conversion en tention
  RS_gas = (5.0-sensor_grove_volt)/sensor_grove_volt; // TAUX VARIATION RESISTANCE SELON GAZ
  ratio = (RS_gas/R0);  // ratio = RS/R0
  uint32_t data=ratio*100;
  return data;
}

uint32_t read_aime_sensor () //Recuperation de la donnée du capteur AIME
{
  int sensor_aime_Value = analogRead(ADC_SENS);
  sensor_aime_volt=(float)sensor_aime_Value/4096*5.0;
  float R_capteur=(R1*(R2+R3)*Vcc/(R2*sensor_aime_volt)) -R1-R5; //formule justifiée en TD
  return R_capteur;
}

uint32_t read_r_alu () //Recuperation de la donnée du capteur de temparateur (R ALU)
{
  int ADC_R_ALU = analogRead(ADC_R_Alu);
  float ADC_R_ALU_volt=(float)ADC_R_Alu/4096*5.0;
  float R_Alu=ADC_R_ALU_volt*R6/(3.3-ADC_R_ALU_volt);
  return R_Alu;
}

void config_PWM() // configure des fonctionnalités PWM
{
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(PWM_OUT, ledChannel); //Liaison de la chaine du Timer avec la pin utilisée
}

float temperature () // mesure de la temperature du capteur en fonction de R aluminium
{
  float T = a*(read_r_alu()) + b;
  T = T - 273; //conversion en Celcius
  return T;
}

int manage_PID() //Correcteur PID pour maintenir la temprature a 200 degrés celcius
{
  int t_2 = micros();
  e_t = 200 - temperature();
  float e_P = Kp * e_t;
  float e_I = Ki * e_t * delta_t; 
  float e_D = Kd * (e_t - e_t_1) / (delta_t);
  t_2 = t_2;
  e_t_1 = e_t;
  uint8_t C = e_P + e_D + e_I;
  if(C > 255) C = 255; //Pour borner la correction entre 0 et 255
 
  return C; //Duty cycle de la PWM de commande
}

void Cmd_PWM_Poly() //Commande de la resistance de chauffe en PWM
{
  int duty_cycle=manage_PID();
  ledcWrite(ledChannel,duty_cyle);
}

void setup() {
  Serial.begin(9600); //initialisation moniteur serie (ESP - PC)
  config_PWM(); //configuration de la PWM
  Serial2.begin(57600, SERIAL_8N1, RX, TX); //initialisation moniteur serie 2 (LORA - ESP)
  initialize_radio(); //initialiation de la radio
  Serial2.print("sys get hwei\r\n"); // recuperation du HWEUI
  Serial.println(Serial2.readStringUntil('\n'));
  attachInterrupt(INTERRUPT_PIN, manage_interrupt, LOW); //Appel de la fonction d'inerruption
  
}

void loop() {
  grove_data=read_grove_sensor();
  aime_data=read_aime_sensor();
  payload[0] = highByte(grove_data);
  payload[1] = lowByte(grove_data);
  payload[2]=highByte(aime_data);  
  payload[1]=lowByte(aime_data);
  myLora.txBytes(payload,4); 
}
