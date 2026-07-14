#include <ESP32Servo.h>

const int THRUSTER_COUNT = 8;
const int PWM_HZ = 50;
const int NEUTRAL_US = 1500;
const int MIN_US = 1100;
const int MAX_US = 1900;
const unsigned long COMMAND_TIMEOUT_MS = 500;

const int THRUSTER_PINS[THRUSTER_COUNT] = {
  // esc1: thruster 8, front left vectored
  // esc2: thruster 7, front left vertical
  // esc3: thruster 6, front right vectored
  // esc4: thruster 5, front right vertical
  // esc5: thruster 1, back left vectored
  // esc6: thruster 2, back left vertical
  // esc7: thruster 3, back right vertical
  // esc8: thruster 4, back right vectored
  21, 19, 27, 18, 5, 14, 12, 26
};

Servo thrusters[THRUSTER_COUNT];
String input_line = "";
unsigned long last_command_ms = 0;

int clamp_pwm(int value) {
  if (value < MIN_US) {
    return MIN_US;
  }
  if (value > MAX_US) {
    return MAX_US;
  }
  return value;
}

void write_all_neutral() {
  for (int i = 0; i < THRUSTER_COUNT; i++) {
    thrusters[i].writeMicroseconds(NEUTRAL_US);
  }
}

void handle_pwm_line(String line) {
  line.trim();
  if (line == "PING") {
    Serial.println("PONG");
    return;
  }
  if (line == "NEUTRAL") {
    write_all_neutral();
    last_command_ms = millis();
    Serial.println("OK NEUTRAL");
    return;
  }
  if (!line.startsWith("PWM ")) {
    Serial.println("ERR UNKNOWN");
    return;
  }

  int values[THRUSTER_COUNT];
  int start = 4;
  for (int i = 0; i < THRUSTER_COUNT; i++) {
    int space = line.indexOf(' ', start);
    String token;
    if (space < 0) {
      token = line.substring(start);
    } else {
      token = line.substring(start, space);
    }

    if (token.length() == 0) {
      Serial.println("ERR SHORT");
      return;
    }

    values[i] = clamp_pwm(token.toInt());
    if (space < 0 && i < THRUSTER_COUNT - 1) {
      Serial.println("ERR SHORT");
      return;
    }
    start = space + 1;
  }

  for (int i = 0; i < THRUSTER_COUNT; i++) {
    thrusters[i].writeMicroseconds(values[i]);
  }
  last_command_ms = millis();
  Serial.println("OK PWM");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < THRUSTER_COUNT; i++) {
    thrusters[i].setPeriodHertz(PWM_HZ);
    thrusters[i].attach(THRUSTER_PINS[i], MIN_US, MAX_US);
  }

  write_all_neutral();
  last_command_ms = millis();
  Serial.println("TARDIGRADE_ESP_THRUSTER_PWM_READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handle_pwm_line(input_line);
      input_line = "";
    } else if (c != '\r') {
      input_line += c;
    }
  }

  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    write_all_neutral();
  }
}
