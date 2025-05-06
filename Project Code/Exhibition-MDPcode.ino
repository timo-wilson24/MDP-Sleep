// PURPOSE: To have a simple way of controlling two pumps, one button and two sensors to demonstrate stuff on my project
#define FSR A2
#define switch_LED 10
#define Soft_Pump 9
#define Button 7
#define LED 6

int forceSensorVal;
int btnReading;
int BtnState;
int lastBtnState;
int LED_PWM;  // setting this higher means it is slower to turn off
int initialLEDPWM = 180;
int delayLEDfade = 1500;
int LED_Value;
int wait_Delay = 16; //Time that we will wait for the switch to go on after initial press
bool pwrState = 0;
bool wake;
int soft_pumpOnTime = 6;  // in seconds
int switch_LEDOffTime = 12;  // in seconds
unsigned long currentTime;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long LEDsStarted = 0;       // counts how long the LEDs have been on
unsigned long debounceDelay = 100;   // the debounce time; increase if the output flickers
unsigned long pumpStarted;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(Soft_Pump, OUTPUT);
  pinMode(switch_LED, OUTPUT);
  pinMode(FSR, INPUT);
  Serial.begin(9600);
  Serial.println("Prototype exhibition display for Tim Wilson Major Work. 23/8/24");
}

void loop() {
  currentTime = millis();
  // put your main code here, to run repeatedly:
  DebounceBtn();  // starts timer for pumpStarted
  PumpOn();       // turns pump on, if the time is less than the duration it should be on for
  ForceSensor();  //turns on LED when force hit
}

void ForceSensor() {
  forceSensorVal = analogRead(FSR);
  Serial.print("Force: ");
  Serial.println(forceSensorVal);
  delay(2);
  LED_Value = map(forceSensorVal, 0, 1023, 0, 255);
  analogWrite(LED,LED_Value);
}


void PumpOn() {
  if ((currentTime - pumpStarted) <= soft_pumpOnTime * 1000) {
    digitalWrite(Soft_Pump, HIGH);
  } else {
    digitalWrite(Soft_Pump, LOW);
  }
  if ((currentTime - pumpStarted) <= wait_Delay * 1000) {
    digitalWrite(switch_LED, LOW);
  } else {
    digitalWrite(switch_LED, HIGH);
  }
}

void DebounceBtn() {  // outputs device mode from 0 to 3, 0 is power off.
  btnReading = digitalRead(Button);
  // Serial.println(modebtnReading);
  if (btnReading != lastBtnState) {
    lastDebounceTime = currentTime;
  }
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce delay, so take it as the actual current state:
    if (btnReading != BtnState) {
      BtnState = btnReading;
      // change mode
      wake = true;
      if ((currentTime - pumpStarted) > wait_Delay*1000) {
        pumpStarted = currentTime;
        Serial.println("Pumps on now");
      }  // only set new pump started time if it has been 16s since last turned on
    }
  }
  lastBtnState = btnReading;
}