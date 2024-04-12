//arduino code for flipkart 5.0 -> SCARA_Robot
// Using Arduino Mega 2560
//Check: (If Not working)
/*
 * limit switches values
 * 5v DC supply issue(encoder) ***IMP***
 * type conversion issue
 */
  
#include <AccelStepper.h>
#include <math.h>
#include <Servo.h>

// limit switches
#define limitSwitch1 12 // J1 
#define limitSwitch2 13 // J2
//#define limitSwitch3 9 // Z-axis bottom
#define limitSwitch4 A3 // Z-axis top
#define limitSwitch5 A0 // for the limit switch in the end effector (to find when it touches the parcel)

// suction DCV pin
#define dcvPin 24  


// stepper motor pins
AccelStepper stepper1(1, 2, 5);  //J1           // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);   //J2
//AccelStepper stepper3(1, 4, 7);
//AccelStepper stepper4(1, 4, 7);  //z-axis  -> high torque dc motor is used instead of stepper motor for z-axis


//Z-axis Motor Pins:
#define pwm 16    // Speed of motor (0 to 255) (PIN -> AN2)
#define dir 17    // Direction of motor (low - move downwards; high - move upwards)  (PIN -> IN2)
#define ENA 18    // Encoder pin A
#define ENB 19    // Encoder pin B
volatile int pos = 0;    // contains the encoder steps of motor <<(could be "long" if the steps are more)>>


//*********** There is a change in length of link2 (L2) ***********
double L1 = 228; // link 1
double L2 = 312; //291; // L2 = 136.5 + 154.5 // link 2 
double theta1, theta2, z;

long stepper1Position, stepper2Position, stepper3Position, zpos;  // before double was used

const float theta1AngleToSteps = 44.444444; //old = 44.444444 
const float theta2AngleToSteps = 35.555555; //old = 35.555555 
//const float zDistanceToSteps = 200; // WIP // 800 steps per mm -> according to calculation (total lenght of workspace in z-axis is 553mm) but 463 from bottom >>> 200 final
// 1 rev -> 1.6mm movement in lead screw
// 1 rev -> 350 ticks(WIP)
const float zDistanceToSteps = 218.75;  // 218.75 ticks -> 1mm (movement)

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


// ********** Flipping Mechanism **********
/*
 1.dcv controlled by 2 relay's
 2.Servo motor for orientation
*/
// Pins:
#define flipdcvPin1 22  // push the piston
#define flipdcvPin2 23  // contract the piston 
Servo myservo;          // servo for orientation
#define servoPin 9  


void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(1);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.print("Serial connected to python");
  
  // Limitswitch Pins
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);
  pinMode(limitSwitch5, INPUT_PULLUP);

  // DCV Pin (Suction & Flip mechanism)
  pinMode(dcvPin, OUTPUT); 
  pinMode(flipdcvPin1,OUTPUT);
  pinMode(flipdcvPin2,OUTPUT);

  // Servo Pin
  myservo.attach(servoPin);
  
  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  //stepper4.setMaxSpeed(15000);
  //stepper4.setAcceleration(2000);

  // Z-axis motor pin setup
  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode(ENA,INPUT);
  pinMode(ENB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENA),readEncoder,RISING);  // attaching interrupt to digital pin 18(ENA)

  delay(500);
  //data[5] = 450; // z-axis (after homing the bot moves to height 450)
  homing();
}

void loop() {
  // getting data from python script: 
  while (Serial.available() == 0){
    //pass
  }
  content = Serial.readStringUntil('\n');  // Read the incoming data from Processing
  cont_from_py = content;   //Read from Python
  // Extract the data from the string and put into separate integer variables (data[] array)
  for (int i = 0; i < 10; i++) {
    int index = content.indexOf(","); // locate the first ","
    data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
    content = content.substring(index + 1); //Remove the number from the string
  }
  /*
   data[0] - Orientation Process ON(1) OFF(2)                  // old -> SAVE / CLEAR button status //data[0] = 2 -> CLEAR //data[0] = 3 -> Homing()
   data[1] - Flip Mechanism (flipdcvPin 1 and 2 start -> 2) [startFlip -> 1 stopFlip -> 0]    // OLD -> RUN / STOP button status  //data[1] = 0 -> STOP
   data[2] - Joint 1 angle
   data[3] - Joint 2 angle
   data[4] - Suction value (dcvPin ON -> 1) ; (dcvPin OFF -> 0)  // before it was Joint 3 angle
   data[5] - Z position  [ 1->upwards(4000) ; 2->downwards(limitswitch) ; 3->downwards(drop area) ; 0->no movement]
   data[6] - Servo value
   data[7] - Speed value
   data[8] - Acceleration value
  */
  // homing from python
  /*if (data[0] == 3){
    homing();
    data[0] = 0;
  }*/
  
  // Movement: 
  stepper1Position = int(data[2] * theta1AngleToSteps);
  stepper2Position = int(data[3] * theta2AngleToSteps);
  //stepper3Position = data[4] * phiAngleToSteps;
  zpos = data[5] * zDistanceToSteps;
  //Serial.print(stepper4.currentPosition());
  
  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  //stepper3.setSpeed(data[7]);
  //stepper4.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  //stepper3.setAcceleration(data[8]);
  //stepper4.setAcceleration(data[8]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  //stepper3.moveTo(stepper3Position);
  //stepper4.moveTo(stepper4Position);
  //Serial.print(data[5] * zDistanceToSteps);
  
  // First movement of joints (L1 and L2):   
  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position) { 
    //if (limitSwitch1 == 1 || limitSwitch2 == 1){
    //  break;
    //}
    //Serial.println(stepper2.currentPosition());
    stepper1.run();
    stepper2.run();
  }
  
  // Second movement of z-axis (stepper motor)
  /*while (stepper4.currentPosition() != stepper4Position){
    //Serial.println(stepper4.currentPosition());
    //if (limitSwitch4 == 1){
     // break;
    //}
    stepper4.run();
  }*/

  // Second movement of z-axis (DC motor)
  // **** IMP ****
  /* 
   *  zpos -> target position
   *  pos -> current position
   */
  if (data[5] == 1){                     //(zpos < pos){  // move upwards
    analogWrite(pwm,150);
    digitalWrite(dir,HIGH);
    while (pos > 2000){  
      // pass
    }
    analogWrite(pwm,0);
  }
  else if (data[5] == 2){                                         //(zpos > pos && digitalRead(limitSwitch5) != 1){    // move downwards
    analogWrite(pwm,150);
    digitalWrite(dir,LOW);      
    while (digitalRead(limitSwitch5) != 1){         //zpos > pos){         // Here mostly check for limitswitch in end-effector to click
      // pass
    }
    analogWrite(pwm,0);
    //zpos = pos;
  }
  else if (data[5] == 3){            // drop zone   // initially the robot must be at height 4000 (i.e, at 1) to execute this code
    analogWrite(pwm,150);
    digitalWrite(dir,LOW);
    while (6000 > pos){       // drop zone at height 8000
      // pass  
    }
    analogWrite(pwm,0);
  }
 
  // Servo Control:
  myservo.write(data[6]);
  delay(100);
  
  // DCV control
  if (data[4] == 1){
    digitalWrite(dcvPin,HIGH);
    //Serial.print("DCVon");
  }
  else{
    digitalWrite(dcvPin,LOW);
    //Serial.print("DCVoff");
  }

  // FlipDCV control
  if (data[1] == 1){
    startFlip();
  }
  
  
  // Confirmation for the movement
  Serial.print(cont_from_py);
  //Serial.print(stepper4.currentPosition());
  delay(200);
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

// Adjust the delay according to the piston speed ******** IMP *******
// Flipping DCV Control:
void startFlip(){
  delay(100);
  digitalWrite(flipdcvPin1,HIGH); 
  delay(2000);
  digitalWrite(flipdcvPin1,LOW);
  delay(2000);
  digitalWrite(flipdcvPin2,HIGH);
  delay(2000);
  digitalWrite(flipdcvPin2,LOW);
  delay(2000);
}


// readEncoder() reads the encoder pulses:
void readEncoder(){
  int b = digitalRead(ENB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
  //Serial.println(posi);
}


void homing() {
  //Serial.print("Inside homing()");
  // Homing Stepper4                   (stepper motor)
  /*while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);  // old -> 1500
    stepper4.runSpeed();
    stepper4.setCurrentPosition(91900); // When limit switch pressed set position to 0 steps     // old->17000 , 55300 -->1mm = 200
  }
  delay(20);
  stepper4.moveTo(90000);
  while (stepper4.currentPosition() != 90000) {
    stepper4.run();
  }*/


  // Homing DC motor (z-axis)  
  analogWrite(pwm,150);
  digitalWrite(dir,HIGH);
  while (digitalRead(limitSwitch4) != 0) {
    // pass
  }
  //Serial.println(pos);
  analogWrite(pwm,0);    // Stops the motor
  pos = 0;               // Set the current positon to 0
  delay(20);

  analogWrite(pwm,150);
  digitalWrite(dir,LOW);
  while (pos <= 4000){   // moving to step 4000
    // pass
  }
  analogWrite(pwm,0);


  // Link2 is already in its home state(i.e., it is touching its limitswitch initially)
  stepper2.setCurrentPosition(-5600);
  
   
  // Homing Stepper2
//  while (digitalRead(limitSwitch2) != 0) {
//    stepper2.setSpeed(-1300);
//    stepper2.runSpeed();
//    stepper2.setCurrentPosition(-5600); // When limit switch pressed set position to -5940 steps
//  }
//  delay(20);

  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1
  while (digitalRead(limitSwitch1) != 0) { //old->1
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-6100); // When limit switch pressed set position to 0 steps //old-> -4000
  }
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
  //Serial.print(stepper4.currentPosition());
  Serial.print("Homed");
}
