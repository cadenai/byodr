#include <ros.h>
#include <Servo.h>
#include <PinChangeInt.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define HALL_IN_PIN A1
#define STEERING_OUT_PIN 5
#define THROTTLE_OUT_PIN 4

// For odometry.
#define H_RPS_MOMENT 0.10

volatile byte h_up;
volatile float h_val, h_rps;
volatile uint32_t h_detect_time, h_publish_time;

// Time the last command was received on the throttle channel.
volatile uint32_t lastCmdReceivedTime;

ros::NodeHandle nodeHandle;
geometry_msgs::TwistStamped message;

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
  message.twist.linear.y = h_rps;
  message.twist.linear.z = analogRead(HALL_IN_PIN);
  publisher.publish(&message);
}

void hall_detect() {
  if (digitalRead(HALL_IN_PIN) == HIGH) {
    h_up++;
    if (h_up >= 2) {
      h_val = 1000000.0 / (micros() - h_detect_time);
      h_rps = H_RPS_MOMENT * h_val + (1.0 - H_RPS_MOMENT) * h_rps;
      h_detect_time = micros();
      h_up = 0;
    }
  }
}

void setup() {
  Serial.begin(57600);

  h_up = 0;
  h_val = 0;
  h_rps = 0;
  h_detect_time = 0;
  h_publish_time = 0;
  
  lastCmdReceivedTime = 0;

  pinMode(HALL_IN_PIN, INPUT_PULLUP);
  
  servoSteering.attach(STEERING_OUT_PIN);
  servoThrottle.attach(THROTTLE_OUT_PIN);

  PCintPort::attachInterrupt(HALL_IN_PIN, hall_detect, CHANGE);

  nodeHandle.initNode();
  nodeHandle.subscribe(subscriber);
  nodeHandle.advertise(publisher);
}

void loop() {
  // Stop the vehicle when command communication slows down or stops functioning.
  if (micros() - lastCmdReceivedTime > 100000) {
    servoThrottle.write(90);
  }

  // Process the odometer.
  // Drop to zero when stopped.
  if (micros() - h_detect_time > 500000) {
    h_rps = (1.0 - H_RPS_MOMENT) * h_rps;
  } 

  // Avoid flooding the topic.
  if (micros() - h_publish_time > 50000) {
    publish_odometer();
    h_publish_time = micros();
  }

  nodeHandle.spinOnce();
}
