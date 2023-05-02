#include <Arduino.h>
#include "BotFCTUC.h"

const int threshold = 300; // Altera este valor consoante o teu sensor!

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    Serial.println("Teste sensor chama e buzzer");

    jeff.begin();
}

void loop() {
    int intensity = jeff.getFlameValue();

    Serial.println("Valor sensor: " + String(intensity));

    Serial.print("Chama detetada :");

    if (intensity < threshold) {
        Serial.println("Sim!");
        jeff.buzzerPlay(40);
    } else {
        Serial.println("Nao!");
        jeff.buzzerPlay(0);
    }

    delay(250);
}