#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// Time the last command was received on the throttle channel.
volatile uint32_t lastCmdReceivedTime, lastHallSentTime = 0;

ros::NodeHandle nodeHandle;
geometry_msgs::TwistStamped message;

// Servo objects generate the signals expected by the Electronic Speed Controller (ESC) and steering servo.
Servo servoThrottle;
Servo servoSteering;

int hallEffect, hallState, hallCount = 0;

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

void publish_odometer(int value) {
  message.header.stamp = nodeHandle.now();  
  message.twist.linear.y = value;
  publisher.publish(&message);
}

void setup() {
  Serial.begin(57600);

  servoThrottle.attach(9);
  // Pin 5 on aav01 and 10 on r3.
  servoSteering.attach(5);
      
  nodeHandle.initNode();
  nodeHandle.subscribe(subscriber);
  nodeHandle.advertise(publisher);
}

void loop() {
  // Stop the vehicle when command communication slows down or stops functioning.
  // Convert to ms.
  if ((micros() - lastCmdReceivedTime) / 1000 > 500) {
    servoThrottle.write(90);
  }

  nodeHandle.spinOnce();

  // Process the odometer.
  hallEffect = analogRead(A0);
  // The threshold is determined empirically.
  if (hallEffect > 250) {
    if (hallState == 0) {
      hallState = 1;
      hallCount++;
    }
  } else {
    hallState = 0;
  }

  // Publish the counter every 100 ms.
  if ((micros() - lastHallSentTime) / 1000 > 100) {
    publish_odometer(hallCount);
    lastHallSentTime = micros();
    hallCount = 0;
  }
}
