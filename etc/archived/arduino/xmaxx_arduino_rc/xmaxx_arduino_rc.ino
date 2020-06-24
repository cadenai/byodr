#include <TimedAction.h>
#include <ros.h>
#include <Servo.h>
#include <PinChangeInt.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// Assign your channel in pins.
#define THROTTLE_IN_PIN A1
#define STEERING_IN_PIN A0

// Assign your channel out pins.
#define THROTTLE_OUT_PIN 6
#define STEERING_OUT_PIN 5

#define analogLow 988
#define analogHigh 1988
#define analogCenter 1488
#define analogCenterLow 1458
#define analogCenterHigh 1518

// Time to wait each loop.
#define SPIN_DELAY 2

// Single logging function timer.
#define TELEMETRY_ACTION_DELAY 30

// In keyboard control mode set throttle to zero on message receiver timeout.
#define CMD_RECEIVED_TIMEOUT 500

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals.
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// Time the last command was received on the throttle channel.
volatile uint32_t lastCmdReceivedTime;

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

// control source {1: human-radio, 2: human-keyboard, 3: computer}
// Read the values from the pins in radio control mode only.
volatile uint8_t control_mode;

ros::NodeHandle nodeHandle;
geometry_msgs::TwistStamped message;

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoThrottle;
Servo servoSteering;

TimedAction cmdLogAction = TimedAction(TELEMETRY_ACTION_DELAY, publish_telemetry);

/*
   The interrupt service routine attached to the trottle pin.
*/
void calcThrottle() {
  if (control_mode == 1) {
    // if the pin is high, its a rising edge of the signal pulse, so lets record its value
    if (digitalRead(THROTTLE_IN_PIN) == HIGH) {
      ulThrottleStart = micros();
    }
    else {
      // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
      // this gives use the time between the rising and falling edges i.e. the pulse duration.
      unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
      // use set the throttle flag to indicate that a new throttle signal has been received
      bUpdateFlagsShared |= THROTTLE_FLAG;
    }
  }
}

/*
   The interrupt service routine attached to the steering pin.
*/
void calcSteering() {
  if (control_mode == 1) {
    if (digitalRead(STEERING_IN_PIN) == HIGH) {
      ulSteeringStart = micros();
    }
    else {
      unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
      bUpdateFlagsShared |= STEERING_FLAG;
    }
  }
}

ros::Publisher publisher("/roy_teleop/cmd_logging", &message);

/*
   2 Channels in keyboard mode separate throttle from steering.
    angular.x = channel {0: none, 1: throttle, 2: steering, 3: both}
    angular.y = _
    angular.z = steering
    linear.x = control source {0: use last, 1: human-radio, 2: human-keyboard, 3: computer}
    linear.y = _
    linear.z = throttle
*/
void publish_telemetry() {
  publish_teleop(servoSteering.readMicroseconds(), servoThrottle.readMicroseconds(), (int) control_mode);
}

void publish_teleop(int angle, int throttle, int ctl_source) {
  message.header.stamp = nodeHandle.now();
  message.twist.angular.z = angle;
  message.twist.linear.x = ctl_source;
  message.twist.linear.z = throttle;
  publisher.publish(&message);
}

/*
   Switch off throttle on command receiver timeout when not in radio mode.
*/
void check_throttle() {
  if (control_mode == 1) {
    return;
  }

  if ((micros() - lastCmdReceivedTime) / 1000 > CMD_RECEIVED_TIMEOUT) {
    if (servoThrottle.readMicroseconds() != analogCenter) {
      servoThrottle.writeMicroseconds(analogCenter);
    }
  }
}

/*
  Drive the servos by radio control.
*/
void rcDrive() {
  if (control_mode != 1) {
    return;
  }

  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;

  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared) {
    // turn interrupts off quickly while we take local copies of the shared variables
    noInterrupts();

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    if (bUpdateFlags & THROTTLE_FLAG) {
      unThrottleIn = unThrottleInShared;
    }

    if (bUpdateFlags & STEERING_FLAG) {
      unSteeringIn = unSteeringInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop

  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.

  // we are checking to see if the channel value has changed, this is indicated
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.

  if (bUpdateFlags & THROTTLE_FLAG) {
    if (servoThrottle.readMicroseconds() != unThrottleIn) {
      servoThrottle.writeMicroseconds(unThrottleIn);
    }
  }

  if (bUpdateFlags & STEERING_FLAG) {
    if (unSteeringIn > analogCenterLow && unSteeringIn < analogCenterHigh) {
      centerSteering();
    }
    else if (servoSteering.readMicroseconds() != unSteeringIn) {
      servoSteering.writeMicroseconds(unSteeringIn);
    }
  }

  bUpdateFlags = 0;
}

/*
   Ros command topic listener function.
   Drives the servos with the command values.
   (use-cases:
      input toggle control = switch joystick on or off
      input drive = throttle and steering
    )

   2 Channels in keyboard mode separate throttle from steering.
    angular.x = channel {0: none, 1: throttle, 2: steering, 3: both}
    angular.y = _
    angular.z = steering
    linear.x = control source {0: use last, 1: human-radio, 2: human-keyboard, 3: computer}
    linear.y = _
    linear.z = throttle
*/
void msgDrive(const geometry_msgs::Twist& msg) {
  // Use the current control setting unless change is required.
  int _control = (int) msg.linear.x;
  if (_control > 0) {
    control_mode = _control;
  }

  // Abort cmd processing when on radio.
  if (control_mode == 1) {
    return;
  }

  uint16_t channel = (uint16_t) msg.angular.x;
  uint16_t angle = (uint16_t) msg.angular.z;
  uint16_t throttle = (uint16_t) msg.linear.z;

  // Drive the servos depending on which channel is active.
  if (channel == 1 || channel == 3) {
    lastCmdReceivedTime = micros();
    if (servoThrottle.readMicroseconds() != throttle) {
      // Protect against malformed cmd values.
      if (throttle > analogHigh) {
        throttle = analogHigh;
      } else if (throttle < analogLow) {
        throttle = analogLow;
      }
      servoThrottle.writeMicroseconds(throttle);
    }
  }

  if (channel == 2 || channel == 3) {
    if (servoSteering.readMicroseconds() != angle) {
      // Protect against malformed cmd values.
      if (angle > analogHigh) {
        angle = analogHigh;
      } else if (angle < analogLow) {
        angle = analogLow;
      }
      servoSteering.writeMicroseconds(angle);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> subscriber("/roy_teleop/cmd_vel", &msgDrive);

void centerSteering() {
  servoSteering.writeMicroseconds(analogCenter);
}

void setup() {
  Serial.begin(57600);

  servoSteering.attach(STEERING_OUT_PIN);
  servoThrottle.attach(THROTTLE_OUT_PIN);

  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering, CHANGE);
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);

  centerSteering();
  delay(300);

  control_mode = 1;

  nodeHandle.initNode();
  nodeHandle.subscribe(subscriber);
  nodeHandle.advertise(publisher);
}

void loop() {
  rcDrive();
  nodeHandle.spinOnce();
  check_throttle();
  cmdLogAction.check();
  delay(SPIN_DELAY);
}




















