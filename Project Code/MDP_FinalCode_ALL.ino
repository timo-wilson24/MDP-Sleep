// Default Values
float defaultBreathPeriod = 4.1;  // set this to be the default period of the breathing motoion of the robot.
int delayLEDfade_default = 3000;  // milliseconds that the LEDs will stay on before fading
int turn_off_device = 6;          // in minutes

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
//Add relevant libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <PeakDetection.h>
#include <DataTome.h>
// Declarations for libaries as to what functions, devices are being used
Adafruit_BME680 bme;  // I2C
PeakDetection peakDetection;

// Hugging stuff variables:
unsigned long timerPumpStart = 0;
int default_pump_on_time = 30;  //In seconds
int pump_on_time;
bool pumpOn;
int forceSensorVal;

// Limit Switch feedback variables
unsigned long motion_number = 1;
unsigned long last_motion_number = 0;
unsigned long timeOfLastBM;
unsigned long timeBetweenBM;
unsigned long LAST_timeBetweenBM;
bool BMlimit_stateLast;
bool BMlimit_state;
float AVG_timeBM;

// Breathing Motion Sensing and output variables:
int peakTVOC;  // state of peak or not peak
double dataCO2;
double dataTVOC;
double filteredTVOC;
int peakDetectLag = 18;  //change this to alter the amount of readings that the peak detection function looks at. Max value before crashing code is
unsigned long reading = 0;
unsigned long timeOfLast;
unsigned long timeBetweenBreaths;
unsigned long timeOfLastIn;
unsigned long timeBetweenBreathsIn;
unsigned long currentTime;
int previousPeakTVOC = 0;
unsigned long movingBreathAvg;  // Continuously updated moving average of the average time of a breath in or out. IN TENTHS OF A SECOND!
int movingAvgDataPoints = 6;
double peakDetectEpsilon = 1.8;
int peakReadingNo = 0;
int motorDutyCycle;
int motorDutyCycle_previous;
bool hitSensor = false;

bool sensorStartFail = false;
double public_targetBMPeriod;
int add_DC_counter = 0;
int minus_DC_counter = 0;

int BM_multiplier = 1152;
int BM_intercept = 2;

DataTomeMvAvg<unsigned, unsigned long> avgBreaths(movingAvgDataPoints);  // use 10 data points for the moving average

//control panel variables
unsigned long time_device_on = 0;
bool wake;
int deviceMode = 1;      // value here will be starting mode
int lastDeviceMode = 3;  // anything that isn't the starting mode
bool pwrState = 1;       // 1 -> device on
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
int delayLEDfade = 1500;

unsigned long lastDebounceTimeMODE = 0;  // the last time the output pin was toggled
unsigned long LEDsStarted = 0;           // counts how long the LEDs have been on
unsigned long debounceDelay = 100;       // the debounce time; increase if the output flickers

//

/* Notes for device modes:
  - Mode 0: Power off mode
  - Mode 1: Breathing only mode
  - Mode 2: Hugging only mode
  - Mode 3: Both modes at once

  Default start mode = 
*/

void setup() {
  Serial.begin(9600);
  while (!Serial) {};
  // initalise all LEDs
  pinMode(pwrLED, OUTPUT);
  pinMode(modeHugLED, OUTPUT);
  pinMode(modeBreathingLED, OUTPUT);
  Serial.println("Device is initalised");
  //intialise button inputs
  pinMode(modeButton, INPUT);
  pinMode(BM_limitswitch, INPUT);
  modeBtnState = digitalRead(modeButton);
  lastmodeBtnState = modeBtnState;  //set the inital state of the button
  LED_PWM = initialLEDPWM;
  delay(5);
  if (!bme.begin()) {
    delay(20);
    Serial.println("Could not find a valid BME688 sensor, check wiring!");
    sensorStartFail = true;
  }
  motorDutyCycle_previous = BM_multiplier / (defaultBreathPeriod + BM_intercept);
  peakDetection.begin(peakDetectLag, 2, .5);  // sets the lag, threshold and influence for Peak Detection algorithm
  // Print settings so that I know what values are being used/changed
  PrintSetup();
  // Set up oversampling and filter initialization
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setGasHeater(320, 150);                   // 320*C for 150 ms
  peakDetection.setEpsilon(peakDetectEpsilon);  // Custom variable I designed changing PeakDetect library. Epsilon -> the absolute integer amount of deviation that becomes the threshold for when a peak could be detected. Means that small deviations/noise in sensor over a long period where there is a very low Std don't get recorded as breaths.
  timeOfLast = millis();                        // Set start time to be recorded as the 'time of last breath'.
}

void loop() {
  // ALWAYS RUNNING [including when device is on standby mode (ie. 'power off')]
  currentTime = millis();
  DebounceMODE();     // outputs deviceMode, pwrState
  ControlPanelLED();  // runs all LEDs on control panel
  TurnOff();          // turns off all functions if time is large
                      // Always home BM when it is in power off or device mode 2 (hugging)
  // Running if device is in 'Power On' state
  if (pwrState) {  // if device is in on mode
    // currentTime = millis();
    if (deviceMode == 1) {  // Breathing motion related modes...
      BM_Feedback();        // collect data from the limit switch as to the current breathing motion info
      BreathingMotion();    // Breathing motion sensor collection, outputs movingBreathAvg...
      analogWrite(pumpSIG, 0);
      delay(2);
    } else if (deviceMode == 2) {
      PumpTime();
      ForceSensor();
      HugUser();
      Home_BM();
    } else if (deviceMode == 3) {
      BM_Feedback();  // collect data from the limit switch as to the current breathing motion info
      PumpTime();
      ForceSensor();
      HugUser();
      BreathingMotion();
      ForceSensor();
    }
  } else if (!pwrState) {  // if in Power off/standby mode
    // Run limit switch checking loop for a minute longer after it has reached power off. This is so that the homing function can still work...
    Home_BM();
    analogWrite(pumpSIG, 0);
    delay(2);
  }
  // loop time average is about 10-11ms
}

// ************************************** //
// FUNCTIONS FOR Breathing Motion
// ************************************** //

void BreathingMotion() {
  BreathingMotionSensor();
  if (movingBreathAvg != 0 && peakReadingNo > 4 && movingBreathAvg < 80) {
    BM_Motor(movingBreathAvg / 10.0);
    public_targetBMPeriod = movingBreathAvg / 10.0;
  } else {
    BM_Motor(defaultBreathPeriod);
    public_targetBMPeriod = defaultBreathPeriod;
  }
  hitSensor = false;
}

void Home_BM() {
  BMlimit_state = digitalRead(BM_limitswitch);
  if (BMlimit_state == 1) {
    analogWrite(motorSIG, 0);
    hitSensor = true;
  }
}

void BM_Feedback() {
  BMlimit_state = digitalRead(BM_limitswitch);
  if (BMlimit_stateLast != BMlimit_state && BMlimit_state == 1) {
    timeBetweenBM = currentTime - timeOfLastBM;  // Calculate elapsed time before last breath
    timeOfLastBM = currentTime;                  // update time of last breath variable
    AVG_timeBM = timeBetweenBM;
    LAST_timeBetweenBM = timeBetweenBM;
    motion_number++;

    if (motion_number >= 2) {  // only when average works
      Serial.print("Cycle number:");
      Serial.print(motion_number);
      Serial.print(", Target Speed:");
      Serial.print(public_targetBMPeriod);
      Serial.print(", Motion timing:");
      Serial.print(AVG_timeBM / 1000.0);
      Serial.print(", Absolute error:");
      Serial.println(abs((public_targetBMPeriod * 1000 - AVG_timeBM)) / 1000.0);
    }
  }
  BMlimit_stateLast = BMlimit_state;
}

void BM_Motor(double targetBMPeriod) {
  if (motion_number != last_motion_number && motion_number >= 3) {
    if (abs((targetBMPeriod * 1000 - AVG_timeBM)) > 100 && abs((targetBMPeriod * 1000 - AVG_timeBM)) < 3000) {  // if absolute value of the error is
      if (targetBMPeriod * 1000 < AVG_timeBM) {                                                                 // if the motor is too slow
        add_DC_counter++;
        motorDutyCycle = motorDutyCycle_previous + abs((targetBMPeriod * 1000 - AVG_timeBM)) / 50;
        Serial.print("Motor PWM = ");
        Serial.print(motorDutyCycle);
        Serial.print(", Adding ");
        Serial.print(int(abs((targetBMPeriod * 1000 - AVG_timeBM)) / 40));
        Serial.println(" duty cycle to speed");
      } else if (targetBMPeriod * 1000 > AVG_timeBM) {  // if motor is too fast
        minus_DC_counter++;
        motorDutyCycle = motorDutyCycle_previous - abs((targetBMPeriod * 1000 - AVG_timeBM)) / 50;
        Serial.print("Motor PWM = ");
        Serial.print(motorDutyCycle);
        Serial.print(", Subtracting ");
        Serial.print(int(abs((targetBMPeriod * 1000 - AVG_timeBM)) / 40));
        Serial.println(" duty cycle from speed");
      }
    } else if (abs((targetBMPeriod * 1000 - AVG_timeBM)) > 1500) {
      motorDutyCycle = BM_multiplier / (targetBMPeriod + BM_intercept);
    }
  } else {
    motorDutyCycle = motorDutyCycle_previous;
  }
  motorDutyCycle_previous = motorDutyCycle;
  if (motorDutyCycle > 255) { motorDutyCycle = 255; }
  analogWrite(motorSIG, motorDutyCycle);
  delay(5);
  last_motion_number = motion_number;
}

void BreathingMotionSensor() {
  if (sensorStartFail) {  // if sensor doesn't start correctly, don't run sensor code, only run default settings.
    // Breathing Motion normal stuff will run at the end of it anyway
  } else {
    // Tell BME688 to begin measurement.
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
      Serial.println("Failed to begin reading :( [Error1: endTime=0]");
      delay(20);
    } else if (!bme.endReading()) {
      Serial.println("Failed to complete reading :( [Error2: endReading is False]");
      delay(20);
    } else {
      dataTVOC = (double)bme.gas_resistance / 1000;  //data variable is set to resistance in KOhms
      peakDetection.add(dataTVOC);                   // Add the data variable to peak detection variable
      reading++;                                     //add to count of readings so far

      if (reading > peakDetectLag) {         // Don't record peaks if the 'lag' data points in the peak detection algo. has not been filled yet. Otherwise peaks could be recorded off little reasoning, throwing off sensor
        peakTVOC = peakDetection.getPeak();  // update variable holding Peak information
        currentTime = millis();              // update current time
        filteredTVOC = peakDetection.getFilt();
        // PrintAllBM(); // custom function to print all variables
        BreathsPM();

        // Put all general code relating to the sensor here!

      } else if (reading == 1) {  //wait for the lag time before doing anything useful (as in the first period the lag has not been filled out)
        Serial.println("Lag time before reading results");
      } else if (reading < peakDetectLag - 4) {  //makes dots in serial so you can tell lag time is being filled
        Serial.print(".");
      } else if (reading < peakDetectLag - 3) {
        Serial.println(".");
      }
    }
    // delay(5); // is delay necessary? Does the sensor get overloaded?
  }
}


void BreathsPM() {                                                                //function to output breathsPM
  if (peakTVOC == -1 && previousPeakTVOC != -1) {                                 // Edge detection - 'catches falling edge' (so that if still in peak, if statement below runs once)
    timeBetweenBreaths = currentTime - timeOfLast;                                // Calculate elapsed time before last breath
    timeOfLast = currentTime;                                                     // update time of last breath variable
    if (timeBetweenBreaths < 11000 && timeBetweenBreaths / 1000 > 2) {            // only measures the average of time between breaths that are less than 11 seconds and less than 2 seconds
      avgBreaths.push(timeBetweenBreaths / 100); // Make the moving average input be the elapsed time in tenths of a second
      movingBreathAvg = avgBreaths.get();
      peakReadingNo++;
      Serial.print("Avg time between breaths: ");
      Serial.print(movingBreathAvg / 10.0);
      Serial.print("s, Time between last reading: ");
      Serial.print(timeBetweenBreaths / 1000.0);
      Serial.println("s");
    } else if (timeBetweenBreaths / 1000 > 11) {
      delay(2);
    } else if (timeBetweenBreaths / 1000 < 2) {
      delay(2);
    }
  } else if (peakTVOC == 1 && previousPeakTVOC != 1) {
    timeBetweenBreathsIn = currentTime - timeOfLastIn;  // Calculate elapsed time before last breath
    timeOfLastIn = currentTime;
    if (timeBetweenBreathsIn < 9000 && timeBetweenBreathsIn / 1000 > 2) {           // only measures the average of time between breaths that are less than 11 seconds and less than 2 seconds
      avgBreaths.push(timeBetweenBreathsIn / 100);
      movingBreathAvg = avgBreaths.get();
      peakReadingNo++;
      Serial.print("Avg time between breaths: ");
      Serial.print(movingBreathAvg / 10.0);
      Serial.print("s, Time between last reading: ");
      Serial.print(timeBetweenBreathsIn / 1000.0);
      Serial.println("s");
    } else if (timeBetweenBreathsIn / 1000 > 11) {
      delay(2);
    } else if (timeBetweenBreathsIn / 1000 < 2) {
      delay(2);
    }
  } 
  previousPeakTVOC = peakTVOC;  // update previous peak value
}

void PrintAllBM() {
  Serial.print("Max:");  // scale for y axis of serial plotter
  Serial.print("220");
  Serial.print(", ");
  Serial.print("PeakTVOC:");
  Serial.print(peakTVOC * 100);
  Serial.print(", ");
  // Serial.print("filteredTVOC:"); //moving average filter
  // Serial.print(filteredTVOC);
  // Serial.print(",");
  Serial.print("Gas:");
  Serial.print(dataTVOC);
  Serial.print(", ");
  Serial.print("Breath Rate:");
  Serial.println(movingBreathAvg / 10.0);
}

// ************************************** //
// FUNCTIONS FOR HUGGING MOTION
// ************************************** //

void PumpTime() {
  if (pumpOn) {
    if (currentTime - timerPumpStart <= pump_on_time * 2000 && timerPumpStart != 0) {  // only run for a short time if it has just been on
      pump_on_time = 6;
      // Serial.println("Running shorter");
    } else if (currentTime - timerPumpStart >= pump_on_time * 2000) {  // if the last time it ran was a while ago, still run for 15
      pump_on_time = 15;
    } else {
      pump_on_time = default_pump_on_time;
    }
  }
}
void HugUser() {
  if (pumpOn) {
    timerPumpStart = currentTime;
    // Serial.print("Pump started at:");
    // Serial.println(timerPumpStart / 1000); // (in seconds)
    analogWrite(pumpSIG, 255);
    delay(2);
    pumpOn = false;
  }
  if (currentTime - timerPumpStart >= pump_on_time * 1000) {
    analogWrite(pumpSIG, 0);
    // Serial.println("Pump off");
    delay(2);
  }
}

void ForceSensor() {
  forceSensorVal = analogRead(force_OUT);
  if (forceSensorVal >= 400) {
    pumpOn = true;
    pump_on_time = 10;
    Serial.println("Force hit!");
  }
}

// ************************************** //
// FUNCTIONS FOR CONTROL PANEL
// ************************************** //

void DebounceMODE() {  // outputs device mode from 0 to 3, 0 is power off.
  modeBtnReading = digitalRead(modeButton);
  // Serial.println(modeBtnReading);
  if (modeBtnReading != lastmodeBtnState) {
    lastDebounceTimeMODE = currentTime;
  }
  if ((currentTime - lastDebounceTimeMODE) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce delay, so take it as the actual current state:
    if (modeBtnReading != modeBtnState) {
      modeBtnState = modeBtnReading;
      // change mode
      if (deviceMode < 3) {
        deviceMode++;
        pwrState = 1;
        wake = true;
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
      pumpOn = true;
      digitalWrite(pwrLED, HIGH);
      digitalWrite(modeHugLED, HIGH);
      digitalWrite(modeBreathingLED, LOW);
    } else if (deviceMode == 3) {  // ALL
      Serial.print("Device is on, current mode is: ");
      Serial.println(deviceMode);
      pumpOn = true;
      ledstate_pwrLED = 1;
      ledstate_modeHugLED = 1;
      ledstate_modeBreathingLED = 1;
      digitalWrite(pwrLED, HIGH);
      digitalWrite(modeHugLED, HIGH);
      digitalWrite(modeBreathingLED, HIGH);
    }
    LEDsStarted = currentTime;
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
    if (currentTime >= LEDfade + LEDsStarted && !LED_finished) {
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
    delay(2);
  }
}

void TurnOff() {
  if (wake) {
    time_device_on = currentTime;
    pwrState = 1;
    wake = false;
  } else if (turn_off_device * 60000 <= currentTime - time_device_on && pwrState == 1) {
    deviceMode = 0;
    pwrState = 0;
    Serial.println("Device has timed out, turning off");
    ledstate_pwrLED = 0;
    ledstate_modeHugLED = 0;
    ledstate_modeBreathingLED = 0;
    digitalWrite(modeHugLED, LOW);
    digitalWrite(modeBreathingLED, LOW);
    Flicker(pwrLED, 500);
    digitalWrite(pwrLED, LOW);
  }
}

void PrintSetup() {
  Serial.println("Settings for this test are:");
  Serial.print("-  Peak Detection Lag Amount: ");
  Serial.println(peakDetectLag);
  Serial.print("-  Peak dection algorithm size of epsilon: ");
  Serial.println(peakDetectEpsilon);
  Serial.print("-  Samples relevant for moving avg of breathing rate: ");
  Serial.println(movingAvgDataPoints);
  Serial.print("-  Default time for a breath: ");
  Serial.println(defaultBreathPeriod);
}
