#include <Arduino.h>
#include "BotFCTUC.h"

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    Serial.println("Teste botao e ventoinha");

    jeff.begin();
}

void loop() {
    bool isButtonPressed = jeff.readButton();

    if (isButtonPressed) {
        Serial.println("Ventoinha ON");
        jeff.fanOn(); // Liga a ventoinha
    } else {
        Serial.println("Ventoinha OFF");
        jeff.fanOff(); // Desliga a ventoinha
    }
    delay(250);
}
