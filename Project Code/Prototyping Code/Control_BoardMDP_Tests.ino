// PINS FOR BOARD
#define pumpSIG 3
#define motorSIG 5
#define BM_limitswitch 7
#define modeButton 8
#define modeBreathingLED 9
#define modeHugLED 10
#define pwrLED 11  //connect to PWM
// can have LED pin mapped, but chose not to
#define PRS_OUT A1
#define force_OUT A2

/* Notes for device modes:
- Mode 0: Power off mode
- Mode 1: Breathing only mode
- Mode 2: Hugging only mode
- Mode 3: Both modes at once

Default start mode = Breathing only

*/

// Limit Switch feedback variables
unsigned long motion_number = 0;
unsigned long timeOfLastBM;
unsigned long timeBetweenBM;
unsigned long LAST_timeBetweenBM;
bool BMlimit_stateLast;
bool BMlimit_state;
float AVG_timeBM;

// 

//control panel variables
int deviceMode = 2;  // value here will be starting mode
int lastDeviceMode = 1;
bool pwrState = 1;  // 1 -> on

bool ledstate_pwrLED;
bool ledstate_modeBreathingLED;
bool ledstate_modeHugLED;
bool LED_finished = 0;
int modeBtnReading;
int modeBtnState;
int lastmodeBtnState;
int LED_PWM;  // setting this higher means it is slower to turn off
int initialLEDPWM = 180;
// unsigned long as millis is big number
int delayLEDfade = 3000;
int delayLEDfade_default = 3000;         // milliseconds that the LEDs will stay on before fading
unsigned long lastDebounceTimeMODE = 0;  // the last time the output pin was toggled
unsigned long LEDsStarted = 0;           // counts how long the LEDs have been on
unsigned long debounceDelay = 100;       // the debounce time; increase if the output flickers

// 

void setup() {
  Serial.begin(9600);
  // initalise all LEDs
  pinMode(pwrLED, OUTPUT);
  pinMode(modeHugLED, OUTPUT);
  pinMode(modeBreathingLED, OUTPUT);
  Serial.println("Device is initalised");
  //intialise button inputs
  pinMode(modeButton, INPUT);
  lastmodeBtnState = digitalRead(modeButton);  //set the inital state of the button
  LED_PWM = initialLEDPWM;
}

void loop() {
  // ALWAYS RUNNING [including when device is on standby mode (ie. 'power off')]
  DebounceMODE();  // outputs deviceMode, pwrState
  ControlPanelLED(); // runs all LEDs on control panel

  // Running if device is in 'Power On' state
  if(pwrState) { // if device is in on mode
    
  } else if (!pwrState) { // if in Power off/standby mode

  }
}

// ************************************** //
// FUNCTIONS FOR CONTROL PANEL
// ************************************** //

void DebounceMODE() {  // outputs device mode from 0 to 3, 0 is power off.
  modeBtnReading = digitalRead(modeButton);
  // Serial.println(modeBtnReading);
  if (modeBtnReading != lastmodeBtnState) {
    lastDebounceTimeMODE = millis();
  }
  if ((millis() - lastDebounceTimeMODE) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce delay, so take it as the actual current state:
    if (modeBtnReading != modeBtnState) {
      modeBtnState = modeBtnReading;
      // change mode
      if (deviceMode < 3) {
        deviceMode++;
        pwrState = 1;
      } else if (deviceMode == 3) {
        deviceMode = 0;
        pwrState = 0;
      }
    }
  }
  lastmodeBtnState = modeBtnReading;
}

void ControlPanelLED() {
  if (deviceMode != lastDeviceMode) {  //only run this if the device state is changed
    if (deviceMode == 0) {
      Serial.println("Device is off");
      ledstate_pwrLED = 1;
      ledstate_modeHugLED = 0;
      ledstate_modeBreathingLED = 0;
      delayLEDfade = 50;
      digitalWrite(modeHugLED, LOW);
      digitalWrite(modeBreathingLED, LOW);
      Flicker(pwrLED, 300);
      digitalWrite(pwrLED, HIGH);
    } else if (deviceMode == 1) {  // breathing only
      Serial.print("Device is on, current mode is: ");
      Serial.println(deviceMode);
      ledstate_pwrLED = 1;
      ledstate_modeHugLED = 0;
      ledstate_modeBreathingLED = 1;
      digitalWrite(pwrLED, HIGH);
      digitalWrite(modeHugLED, LOW);
      digitalWrite(modeBreathingLED, HIGH);
    } else if (deviceMode == 2) {  // hugging only
      Serial.print("Device is on, current mode is: ");
      Serial.println(deviceMode);
      ledstate_pwrLED = 1;
      ledstate_modeHugLED = 1;
      ledstate_modeBreathingLED = 0;
      digitalWrite(pwrLED, HIGH);
      digitalWrite(modeHugLED, HIGH);
      digitalWrite(modeBreathingLED, LOW);
    } else if (deviceMode == 3) {
      Serial.print("Device is on, current mode is: ");
      Serial.println(deviceMode);
      ledstate_pwrLED = 1;
      ledstate_modeHugLED = 1;
      ledstate_modeBreathingLED = 1;
      digitalWrite(pwrLED, HIGH);
      digitalWrite(modeHugLED, HIGH);
      digitalWrite(modeBreathingLED, HIGH);
    }
    LEDsStarted = millis();
    LED_finished = 0;
    if (deviceMode != 0) {
      delayLEDfade = delayLEDfade_default;

    }
  }
  LEDs_Off(delayLEDfade);
  lastDeviceMode = deviceMode;
}

void Flicker(int pin, int delay_time) {
  digitalWrite(pin, HIGH);
  delay(delay_time);
  digitalWrite(pin, LOW);
  delay(delay_time);
  digitalWrite(pin, HIGH);
  delay(delay_time);
  digitalWrite(pin, LOW);
  delay(delay_time);
}

void LEDs_Off(int LEDfade) {
  if (deviceMode == lastDeviceMode) {  // only run if the LEDs have already turned on
    if (millis() >= LEDfade + LEDsStarted && !LED_finished) {
      if (ledstate_pwrLED == 1) {
        N_B_PowerOffLED(pwrLED);
      }  // only 'turn off' an LED if its on. If not then the LED turns on to the turning off brightness
      if (ledstate_modeHugLED == 1) {
        N_B_PowerOffLED(modeHugLED);
      }
      if (ledstate_modeBreathingLED == 1) {
        N_B_PowerOffLED(modeBreathingLED);
      }
    } else if (LED_finished) {
      LED_PWM = initialLEDPWM;
      // analogWrite(pwrLED, 0);
      // analogWrite(modeHugLED, 0);
      // analogWrite(modeBreathingLED, 0);
    }
  }
}

void N_B_PowerOffLED(int pin) {  //Non blocking alt code
  analogWrite(pin, LED_PWM);
  if (LED_PWM != 0) {
    LED_PWM--;
    LED_PWM--;
    // Serial.println("Start turning LEDs off");
    analogWrite(pin, LED_PWM);
    delay(8);
  } else if (LED_PWM == 0) {
    LED_finished = 1;
    delay(5);
  }
}
