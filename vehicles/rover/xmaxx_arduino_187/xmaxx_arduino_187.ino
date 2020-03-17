#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// Time the last command was received on the throttle channel.
volatile uint32_t lastCmdReceivedTime = 0;

ros::NodeHandle nodeHandle;
geometry_msgs::TwistStamped message;

// For odometry.
int hallSensorPin = 2;
byte h_up, h_rotation;
float h_rpm, h_detect;
unsigned long h_prev;

// Servo objects generate the signals expected by the Electronic Speed Controller (ESC) and steering servo.
Servo servoThrottle;
Servo servoSteering;

/*
   Ros command topic listener drives the servos with the command values.
    angular.x = channel {0: none, 1: throttle, 2: steering, 3: both}
    angular.y = _
    angular.z = steering
    linear.x = control source 
    linear.y = _
    linear.z = throttle
*/
void msgDrive(const geometry_msgs::Twist& msg) {
  uint16_t angle = (uint16_t) msg.angular.z;
  uint16_t throttle = (uint16_t) msg.linear.z;
  lastCmdReceivedTime = micros();
  // Apply ceilings for forward and reverse throttle as a safety measure.      
  if (throttle > 140) {
    throttle = 140;
  } else if (throttle < 40) {
    throttle = 40;
  }
  servoThrottle.write(throttle);
  // Protect the steering margins.
  if (angle > 180) {
    angle = 180;
  } else if (angle < 0) {
    angle = 0;
  }
  if (servoSteering.read() != angle) {
    servoSteering.write(angle);  
  }
}

ros::Subscriber<geometry_msgs::Twist> subscriber("/roy_teleop/command/drive", &msgDrive);

ros::Publisher publisher("/roy_teleop/sensor/odometer", &message);

void publish_odometer() {
  message.header.stamp = nodeHandle.now();  
  message.twist.linear.x = h_up;
  message.twist.linear.y = h_rpm;
  publisher.publish(&message);
}

void setup() {
  Serial.begin(57600);

  h_up = 0;
  h_rpm = 0;
  h_detect = 0;
  h_rotation = 0;
  h_prev = 0;

  pinMode(hallSensorPin, INPUT);   
  
  // Throttle is pin 9.
  servoThrottle.attach(9);
  // Steering is pin 5.
  servoSteering.attach(5);
      
  nodeHandle.initNode();
  nodeHandle.subscribe(subscriber);
  nodeHandle.advertise(publisher);
}

void hall_detect() {
  h_rotation++;
  h_detect = millis();
  if (h_rotation >= 3) {
    h_rpm = (1000 / (millis() - h_prev)) * 60; 
    h_prev = millis();
    h_rotation = 0;
  }
}

void loop() {
  // Stop the vehicle when command communication slows down or stops functioning.
  // Convert to ms.
  if ((micros() - lastCmdReceivedTime) / 1000 > 100) {
    servoThrottle.write(90);
  }

  // Process the odometer.
  if (digitalRead(hallSensorPin) == HIGH) {
    if (h_up == 0) {
      h_up = 1;
      hall_detect();
    }
  } else {
    h_up = 0;
  }

  // Drop to zero when stopped.
  if (millis() - h_detect > 200) {
    h_rpm = 0;
  } 
  publish_odometer();

  nodeHandle.spinOnce();
}
