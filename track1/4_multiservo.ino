#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;
Servo servos[6];  // create servo objects to control servos

void arduino_sub(const std_msgs::Int16MultiArray& msg) {
  if (msg.data_length == 6) { // Check if we received exactly 5 angles
    int angle0 = msg.data[0];
    int angle1 = msg.data[1];
    int angle2 = msg.data[2];
    int angle3 = msg.data[3];
    int angle4 = msg.data[4];
    int angle5 = msg.data[5];

    // Ensure the input is within valid servo angle range (0 to 180)
    if (angle0 >= 0 && angle1 <= 180 &&
        angle1 >= 0 && angle1 <= 180 && 
        angle2 >= 0 && angle2 <= 180 && 
        angle3 >= 0 && angle3 <= 180 && 
        angle4 >= 0 && angle4 <= 180 && 
        angle5 >= 0 && angle5 <= 180) {
      moveServos(angle0, angle1, angle2, angle3, angle4, angle5);
    } else {
      nh.loginfo("Received invalid servo angle.");
    }
  } else {
    nh.loginfo("Received incorrect number of servo angles.");
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("movement", &arduino_sub);

void setup() {
  servos[0].attach(3);
  servos[1].attach(5);  
  servos[2].attach(6);
  servos[3].attach(9);
  servos[4].attach(10);
  servos[5].attach(11);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

void moveServos(int angle0, int angle1, int angle2, int angle3, int angle4, int angle5) {
  servos[0].write(angle0);  // move the servos to the specified angles
  servos[1].write(angle1);
  servos[2].write(angle2);
  servos[3].write(angle3);
  servos[4].write(angle4);
  servos[5].write(angle5);
  delay(15);  // optional delay to make the movement smoother
}
