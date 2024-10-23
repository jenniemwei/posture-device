// Posture Device

  // Reads an infrared temperature sensor and laser distance as input and lights up 3 LEDS as output : 2 leds based on temp &distance values,
  // 1 based on millis timer


  //     pin mapping :

  // Arduino pin | role | description
  //   ___________________________________________________________________
  //           4   OUTPUT  drives LED
  //           5   OUTPUT  drives LED
  //           6   OUTPUT  drives LED



#include <Wire.h>
#include <DFRobot_VL53L0X.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DFRobot_VL53L0X sensor;


int leftLEDPin = 4;
int rightLEDPin = 5;
int farLEDPin = 6;

unsigned long workStartTime = 0;
unsigned long breakStartTime = 0;

int seatedMaxDist = 400; //distance threshold for person to be seated
int bodyMinTemp = 75; //person temp threshold
int postureVal = 200; //distance threshold for acceptable posture
unsigned int sitTime = 20000;  //20 s for demo
unsigned long sitTimeStart = 0;
unsigned long breakTimeStart = 0;
unsigned long sitTimer = 0;

//break and sitting states
bool isBreakReminder = false;
bool sitTimerStarted = false;


void setup() {
  // distance sensor setup
  Serial.begin(9600);
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();

  // temp sensor setup
  mlx.begin();

  // LED setup
  pinMode(leftLEDPin, OUTPUT);
  pinMode(rightLEDPin, OUTPUT);
  pinMode(farLEDPin, OUTPUT);

}
void loop() {
  //read temp and distance sensor values
  int dist = sensor.getDistance();
  int temp = mlx.readObjectTempF();
  //set states for if user is sitting and if posture is bad
  bool isSitting = dist < seatedMaxDist && temp > bodyMinTemp;
  bool postureBad = dist > postureVal;

  if (isSitting) {
    //posture check
    if (postureBad) {
      flashLights();
    }

    if (sitTimerStarted) {
      sitTimer = millis() - sitTimeStart;
      Serial.println((String) "SIT TIME: " + sitTimer);

      if (sitTimer > 500 && !postureBad) {  //turn off timer started indicator lights after .5 s
        digitalWrite(leftLEDPin, LOW); 
        digitalWrite(rightLEDPin, LOW);
      }

      if (sitTimer >= sitTime) {
        isBreakReminder = true;
        Serial.println("BREAK REMINDER");
      }
    } else if (!sitTimerStarted) {
      sitTimeStart = millis();
      sitTimerStarted = true;
      digitalWrite(leftLEDPin, HIGH);  //timer started indicator lights
      digitalWrite(rightLEDPin, HIGH);
      digitalWrite(farLEDPin, LOW);  //turn off break indicator light
    }
  }


  if (isBreakReminder) { //break indicators
    alternateFlash(); //flash left & right lights
    digitalWrite(farLEDPin, HIGH); //break light
  }

  if (!isSitting) {
    digitalWrite(leftLEDPin, LOW); //turn left & right lights off
    digitalWrite(rightLEDPin, LOW);
    isBreakReminder = false; //update reminder state
    sitTimer = 0; //reset sit timer
    sitTimerStarted = false; //reset sit timer state
  }
}

//flashing lights function for fast flashing lights, uses millis so other code is not delayed
void flashLights() {
  static bool ledState = false;
  static unsigned long lastBlinkTime = 0;

  if (millis() - lastBlinkTime >= 50) {  // Blink every 500ms
    ledState = !ledState; //updates current state for blink
    digitalWrite(leftLEDPin, ledState);
    digitalWrite(rightLEDPin, ledState);
    lastBlinkTime = millis();
  }
}
//alternating lights function for the break light, uses millis so other code is not delayed
void alternateFlash() {
  static bool ledState = false;
  static unsigned long lastBlinkTime = 0; 

  if (millis() - lastBlinkTime >= 500) {  // Blink every 500ms
    ledState = !ledState; //updates current state for blink
    digitalWrite(leftLEDPin, !ledState);
    digitalWrite(rightLEDPin, ledState); //left and right opposite states for alternating flash
    lastBlinkTime = millis(); 
  }
}



