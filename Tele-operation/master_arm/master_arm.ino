#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>


// potentiometer pins:
const byte potPin1 = A0; // Link 1
const byte potPin2 = A1; // Link 2


int joint_vals[2];  // two joint potentiometer values are stored in joint_vals array


ros::NodeHandle nh;
std_msgs::Int16MultiArray msg;


// Instantiate publisher (for debugging purposes)
ros::Publisher teleop_joints("joint_vals",&msg);


void setup() {
  // Serial.begin(57600);
  // Pin modes:
  pinMode(potPin1,INPUT);
  pinMode(potPin2,INPUT);

 
  nh.initNode();
  nh.advertise(teleop_joints);
}

void loop() {
  // Read potentioemter values:
  joint_vals[0] = analogRead(potPin1);
  joint_vals[1] = analogRead(potPin2);

  // Note: the negative sign might change depending on potentiometer orientation. !!!WIP!!!
  // Adjust the potentiometer values:
  joint_vals[0] = map(joint_vals[0], 0 ,1023, -6100, 6100);  // limitswitch is at -4000 steps
  joint_vals[1] = map(joint_vals[1], 0 ,1023, -5600, 5600);  // limitswitch is at -5940 steps
  
  // Joint state publish back to ros:
  msg.data=joint_vals;
  msg.data_length = 2;
  teleop_joints.publish(&msg);
  nh.spinOnce();
  
  delay(1);
}
