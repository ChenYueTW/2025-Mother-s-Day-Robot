// 2~5 Coral Level 1~4
// 6, 7 Reef algae High & Low
// 8, 9 AutoIntake
// 10 Funnel
// 11 stop

#include <Joystick.h>

Joystick_ Joystick;

void setup() {
  // Initialize Button Pins
  for (int i = 2; i <= 12; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // Initialize Joystick Library
  Joystick.begin();
  Serial.begin(9600);
}

// Constant that maps the phyical pin to the joystick button.
const int pinToButtonMap = 2;

// Last state of the button
int lastButtonState[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void loop() {
  // Read pin values
  for (int index = 0; index < 11; index++) {
    int currentButtonState = !digitalRead(index + pinToButtonMap);
    Serial.print(currentButtonState);
    if (currentButtonState != lastButtonState[index])
    {
      Joystick.setButton(index, currentButtonState);
      lastButtonState[index] = currentButtonState;
    }
  }
  Serial.println(" ");

  delay(50);
}
