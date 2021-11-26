# 2021_2022_LIEVRE_NGUYEN
Gaz sensor project

## Piste verte (Lora)
Câblage:
-RST sur D21
-Rx sur D19
-Tx sur D18

Si erreur, reboot la ESP32
En Tx:
- rien à changer dans le .ino mais dans le moniteur série, il faut rentrer des commandes
"radio tx" + hexa

En Rx:
- même programme mais ajout dans le loop une ligne
"Serial2.print("radio rx 0\r\n");"
"Serial2.readStringUntil('\n');"

## Piste bleue (LoraWAN)
"mac tx cnf 81"+ hexa
Gas sensor v1.5 (MQ2)
