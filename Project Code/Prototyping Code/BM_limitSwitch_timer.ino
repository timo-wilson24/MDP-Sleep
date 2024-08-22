/*
 Very basic code to time the cycle of the breathing motion and count how many cycles it has done so far. Used primarily for prototyping and testing. 
*/

unsigned long motion_number = 0;
unsigned long timeOfLastBM;
unsigned long timeBetweenBM;
unsigned long LAST_timeBetweenBM;
bool BMlimit_stateLast;
bool BMlimit_state;
float AVG_timeBM;

int motorDutyCycle = 255;  // value output to the PWM (analog out)

#define BM_limitswitch 7
#define motorSIG 5

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  BM_Feedback();
  // Serial.println(BMlimit_state);
  analogWrite(motorSIG, motorDutyCycle); // set duty cycle here... 
  // wait 2 milliseconds before the next loop for the analog-to-digital converter to settle after the last reading:
  delay(2);
}

void BM_Feedback() {
  BMlimit_state = digitalRead(BM_limitswitch);
  if (BMlimit_stateLast != BMlimit_state && BMlimit_state == 1) {
    timeBetweenBM = millis() - timeOfLastBM;  // Calculate elapsed time before last breath
    timeOfLastBM = millis();                  // update time of last breath variable
    AVG_timeBM = (LAST_timeBetweenBM + timeBetweenBM) / 2; // average last two data points [optional kind of step I implemented]
    LAST_timeBetweenBM = timeBetweenBM; 
    motion_number++;
    Serial.print("Cycle number:");
    Serial.print(motion_number);
    Serial.print(", Motion timing:");
    Serial.println(AVG_timeBM / 1000.0);
  }
  BMlimit_stateLast = BMlimit_state;
}
