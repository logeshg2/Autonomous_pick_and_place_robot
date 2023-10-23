//arduino code for flipkart 5.0 -> SCARA_Robot
//Check: (If Not working)
/*
 * limit switches values
 */
  
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// limit switches
#define limitSwitch1 A0 // J1 
#define limitSwitch2 10 // J2
//#define limitSwitch3 9 // J3 -> Not required
#define limitSwitch4 A3 // Z-axis top

// suction DCV pin
#define dcvPin 12  // SpnEn

// stepper motor pins
AccelStepper stepper1(1, 2, 5);  //J1           // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);   //J2
//AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 4, 7);  //z-axis

Servo turnServo; //servo object

double L1 = 228; // link 1
double L2 = 291; // L2 = 136.5 + 154.5 // link 2
double theta1, theta2, z;

double stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444; //old = 44.444444 
const float theta2AngleToSteps = 35.555555; //old = 35.555555 
const float zDistanceToSteps = 200; // WIP // 800 steps per mm -> according to calculation (total lenght of workspace in z-axis is 553mm) but 463 from bottom >>> 200 final

byte inputValue[5];
int k = 0;

String content = "";
String cont_from_py = "";
int data[10];

int theta1Array[100];
int theta2Array[100];
int zArray[100];
int positionsCounter = 0;
int gripperArray[100];

void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(1);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print("Serial connected to python");
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  //pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  pinMode(dcvPin, OUTPUT); // suction pin
  
  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  /* // turning servo:
  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(data[6]);*/

  delay(500);
  data[5] = 450; // z-axis
  homing();
  //Serial.print("End of Setup");                   
}

void loop() {
  // getting data from python script: 
  //delay(10000);
  while (Serial.available() == 0){
    //pass
  }
  //while (Serial.available() > 0) {
  //content = "";
  content = Serial.readStringUntil('\n');  // Read the incoming data from Processing
  cont_from_py = content;   //Read from Python
  // Extract the data from the string and put into separate integer variables (data[] array)
  for (int i = 0; i < 10; i++) {
    int index = content.indexOf(","); // locate the first ","
    data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
    content = content.substring(index + 1); //Remove the number from the string
  }
  Serial.print(cont_from_py);
  /*
   data[0] - SAVE / CLEAR button status //data[0] = 2 -> CLEAR //data[0] = 3 -> Homing()
   data[1] - RUN / STOP button status  //data[1] = 0 -> STOP
   data[2] - Joint 1 angle
   data[3] - Joint 2 angle
   data[4] - Suction value (DCV) ON(1) ; OFF(0)  // before it was Joint 3 angle
   data[5] - Z position
   data[6] - Gripper value
   data[7] - Speed value
   data[8] - Acceleration value
    */
  //}
  //Serial.print("here");
  // homing from python
  /*if (data[0] == 3){
    homing();
    data[0] = 0;
  }*/
  //Serial.print("hello");

  // Movement: 
  stepper1Position = int(data[2] * theta1AngleToSteps);
  stepper2Position = int(data[3] * theta2AngleToSteps);
  //stepper3Position = data[4] * phiAngleToSteps;
  stepper4Position = data[5] * zDistanceToSteps;
  //Serial.print(stepper4.currentPosition());
  
  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  //stepper3.setSpeed(data[7]);
  stepper4.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  //stepper3.setAcceleration(data[8]);
  stepper4.setAcceleration(data[8]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  //stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);
  //Serial.print(data[5] * zDistanceToSteps);
  //First movement of joints (L1 and L2): 
  
  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position) { 
    //if (limitSwitch1 == 1 || limitSwitch2 == 1){
    //  break;
    //}
    //Serial.println(stepper2.currentPosition());
    stepper1.run();
    stepper2.run();       //|| stepper3.currentPosition() != stepper3Position 
    //stepper3.run();
  }
  
  //Second movement of z-axis
  while (stepper4.currentPosition() != stepper4Position){
    //Serial.println(stepper4.currentPosition());
    //if (limitSwitch4 == 1){
     // break;
    //}
    stepper4.run();
  }
  // DCV control
  if (data[4] == 1){
    digitalWrite(dcvPin,HIGH);
    //Serial.print("DCVon");
  }
  else{
    digitalWrite(dcvPin,LOW);
    //Serial.print("DCVoff");
  }
  
  // Confirmation for the movement
  //Serial.print(cont_from_py);
  //Serial.print(stepper4.currentPosition());
  //gripperServo.write(data[6]);
  delay(300);
  //Serial.print("Came here");

}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

void homing() {
  // Homing Stepper4
  //Serial.print("Inside homing()");
  while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(91900); // When limit switch pressed set position to 0 steps     // old->17000 , 55300 -->1mm = 200
  }
  delay(20);
  stepper4.moveTo(90000);
  while (stepper4.currentPosition() != 90000) {
    stepper4.run();
  }

  // Homing Stepper2
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(-1300);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5940); // When limit switch pressed set position to -5440 steps
  }
  delay(20);

  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-4000); // When limit switch pressed set position to 0 steps //old-> -3955
  }
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
  //Serial.print(stepper4.currentPosition());
  Serial.print("Homed");
}
