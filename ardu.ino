const int numButtons = 10;
const int numAnalog = 6;
const int buttonPins[numButtons] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const int analogPins[numAnalog] = {A0, A1, A2, A3, A4, A5};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
}

void loop() {
  // Read button states
  for (int i = 0; i < numButtons; i++) {
    int buttonState = digitalRead(buttonPins[i]);
    Serial.print("B");
    Serial.print(i);
    Serial.print(":");
    Serial.print(buttonState);
    Serial.print(",");
  }

  // Read analog values
  for (int i = 0; i < numAnalog; i++) {
    int analogValue = analogRead(analogPins[i]);
    Serial.print("A");
    Serial.print(i);
    Serial.print(":");
    Serial.print(analogValue);
    if (i < numAnalog - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
  delay(100);
}
