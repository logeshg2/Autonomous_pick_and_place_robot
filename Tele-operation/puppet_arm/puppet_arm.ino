/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * 6-axis robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation on the PC and sent to Arduino via rosserial. It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached.
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <Servo.h> 
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// stepper motor pins
AccelStepper joint1(1, 2, 5);  //J1           // (Type:driver, STEP, DIR)
AccelStepper joint2(1, 3, 6);   //J2

MultiStepper steppers;

//*********** There is a change in length of link2 (L2) ***********
double L1 = 228; // link 1
double L2 = 312; //291; // L2 = 136.5 + 154.5 // link 2 

int joint_step[2];  // 2 -> because there are 2 joints
int joint_status = 0;
int arr[2];

ros::NodeHandle nh;
std_msgs::Int16MultiArray msg;

// Instantiate publisher (for debugging purposes)
ros::Publisher joints("joint_position",&msg);

void arm_cb(const std_msgs::Int16MultiArray& arm_steps) {
  joint_status = 1;
  joint_step[0] = int(arm_steps.data[0]);
  joint_step[1] = int(arm_steps.data[1]);

  // To counter the error:
//  int cur_j1 = joint1.currentPosition();
//  int cur_j2 = joint2.currentPosition(); 
//
//  if ((cur_j1 + 2) >= joint_step[0] && (cur_j1 - 2) <= joint_step[0]){
//    joint_status = 0;
//  }
//  if ((cur_j2 + 2) >= joint_step[1] && (cur_j2 - 2) <= joint_step[1]){
//    joint_status = 0;
//  }
  
//  Serial.println(arr[0]);
}

//void gripper_cb(const std_msgs::UInt16& cmd_msg) {
//  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
//  digitalWrite(13, HIGH - digitalRead(13));  // Toggle LED  
//}


// Instantiate subscribers
ros::Subscriber<std_msgs::Int16MultiArray> arm_sub("joint_vals", arm_cb); // subscribes to joint_vals on arm
//ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); // subscribes to gripper position
// To publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>


void setup() {
  //Serial.begin(115200);
  //pinMode(13, OUTPUT);
  joint_status = 0;

  nh.initNode();
  nh.subscribe(arm_sub);
  //nh.subscribe(gripper_sub);
  nh.advertise(joints);

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(1550);
  joint1.setCurrentPosition(-6100);
  joint2.setCurrentPosition(-5600);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);

  // Configure gripper servo
  //gripper.attach(11);
}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[2];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0];
    positions[1] = joint_step[1];

    steppers.moveTo(positions);
    //nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    //gripper.write(joint_step[5]);  // Move gripper after manipulator reaches the goal   
  }
  
  joint_status = 0;

  arr[0] = joint1.currentPosition();
  arr[1] = joint2.currentPosition();  //!!!WIP!!!
  
  
  // Joint state publish back to ros:
  msg.data = arr;
  msg.data_length = 2;
  joints.publish(&msg);
  nh.spinOnce();
  
  delay(1);
}
