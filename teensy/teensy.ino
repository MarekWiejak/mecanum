#include <ros.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <std_msgs/Bool.h>
#include "src/messages/Vector4float.h"
#include "src/messages/Vector4int.h"
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
geometry_msgs::Twist measured_vel;

// define teeensy pins for motor control:
// run - motor rotation causing robot to move forward
// reverse - motor rotation causing robot to move backward
// encoder_Pin1 and encoder_Pin2 - should be connected in such manner,
//    that value of rpmPV topic are positive when moving forward and
//    negative when moving backward
const int run1 = 7, reverse1 = 6;
  const int encoder1Pin1 = 14, encoder1Pin2 = 15;
const int run2 = 10, reverse2 = 11;
  const int encoder2Pin1 = 16, encoder2Pin2 = 17;
const int run3 = 8, reverse3 = 9;
  const int encoder3Pin1 = 18, encoder3Pin2 = 19;
const int run4 = 4, reverse4 = 5;
  const int encoder4Pin1 = 20, encoder4Pin2 = 21;

const int pulses_per_rotation = 4 * 56 * 11; // 4 * reduction ratio * number of pusles on each sensor per rotation
const long interv = 50; //main loop interv
const float st = 0.5; //settling time (control algorithm)
const int thresh = 30; // threshold of pwm value for motor to start turning
const double count2rpm = interv*pulses_per_rotation/60/1000;

unsigned long last_cmd_time;
unsigned long schedule_time;

Encoder encoder1(encoder1Pin1, encoder1Pin2);
Encoder encoder2(encoder2Pin1, encoder2Pin2);
Encoder encoder3(encoder3Pin1, encoder3Pin2);
Encoder encoder4(encoder4Pin1, encoder4Pin2);

// PID settings
const double Kp_def = 0.685/st, Ki_def = Kp_def/0.125, Kd_def = 0;
const double Kp_1 = Kp_def, Ki_1 = Ki_def, Kd_1 = 0;
const double Kp_2 = Kp_def, Ki_2 = Ki_def, Kd_2 = 0;
const double Kp_3 = Kp_def, Ki_3 = Ki_def, Kd_3 = 0;
const double Kp_4 = Kp_def, Ki_4 = Ki_def, Kd_4 = 0;

double CV1 = 0, CV2 = 0, CV3 = 0, CV4 = 0;
double PV1 = 0, PV2 = 0, PV3 = 0, PV4 = 0;
double SP1 = 0, SP2 = 0, SP3 = 0, SP4 = 0;

// initialize PIDs
PID PID1(&PV1, &CV1, &SP1, Kp_1, Ki_1, Kd_1, DIRECT);
PID PID2(&PV2, &CV2, &SP2, Kp_2, Ki_2, Kd_2, DIRECT);
PID PID3(&PV3, &CV3, &SP3, Kp_3, Ki_3, Kd_3, DIRECT);
PID PID4(&PV4, &CV4, &SP4, Kp_4, Ki_4, Kd_4, DIRECT);



void callback_cmd_vel(const geometry_msgs::Twist& vel){
  vel_2_rpm(vel);
  last_cmd_time = millis();
}

//void check_connection(const ros::TimeEvent& event){
//  
//}

ros::Publisher chatter_vel("measured_vel", &measured_vel);
ros::Subscriber<geometry_msgs::Twist> vel_SP("cmd_vel", &callback_cmd_vel);


void setup() {
  pinMode(run1, OUTPUT);
    pinMode(reverse1, OUTPUT);
  pinMode(run2, OUTPUT);
    pinMode(reverse2, OUTPUT);
  pinMode(run3, OUTPUT);
    pinMode(reverse3, OUTPUT);
  pinMode(run4, OUTPUT);
    pinMode(reverse4, OUTPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  nh.initNode();
  nh.advertise(chatter_vel);
  nh.subscribe(vel_SP);
//  nh.createTimer

  PID1.SetSampleTime(interv);
  PID2.SetSampleTime(interv);
  PID3.SetSampleTime(interv);
  PID4.SetSampleTime(interv);

  PID1.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID2.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID3.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID4.SetOutputLimits(-255 + thresh, 255 - thresh);
  
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  PID3.SetMode(AUTOMATIC);
  PID4.SetMode(AUTOMATIC);

  schedule_time = millis();
}

void loop() {
    int count1=0, count2=0, count3=0, count4=0;
    count1 = encoder1.read();
      encoder1.write(0);
    count2 = encoder2.read();
      encoder2.write(0);
    count3 = encoder3.read();
      encoder3.write(0);
    count4 = encoder4.read();
      encoder4.write(0);

    PV1 = count1 / count2rpm;
    PV2 = count2 / count2rpm;
    PV3 = count3 / count2rpm;
    PV4 = count4 / count2rpm;

    PID1.Compute();
    if ((PID1.GetMode() == AUTOMATIC && SP1 == 0 && abs(PV1) < 2.0) || (PID1.GetMode() == MANUAL && CV1 == 0)){
      analogWrite(run1, 0);
      analogWrite(reverse1, 0);
    }
    else if(CV1 > 0){
      analogWrite(run1, int(CV1) + thresh);
      analogWrite(reverse1, 0);
    }
    else if(CV1 < 0){
      analogWrite(run1, 0);
      analogWrite(reverse1, int(-CV1) + thresh);
    }

    PID2.Compute();
    if ((PID2.GetMode() == AUTOMATIC && SP2 == 0 && abs(PV2) < 2.0) || (PID2.GetMode() == MANUAL && CV2 == 0)){
      analogWrite(run2, 0);
      analogWrite(reverse2, 0);
    }
    else if(CV2 > 0){
      analogWrite(run2, int(CV2) + thresh);
      analogWrite(reverse2, 0);
    }
    else if(CV2 < 0){
      analogWrite(run2, 0);
      analogWrite(reverse2, int(-CV2) + thresh);
    }
    
    PID3.Compute();
    if ((PID3.GetMode() == AUTOMATIC && SP3 == 0 && abs(PV3) < 2.0) || (PID3.GetMode() == MANUAL && CV3 == 0)){
      analogWrite(run3, 0);
      analogWrite(reverse3, 0);
    }
    else if(CV3 > 0){
      analogWrite(run3, int(CV3) + thresh);
      analogWrite(reverse3, 0);
    }
    else if(CV3 < 0){
      analogWrite(run3, 0);
      analogWrite(reverse3, int(-CV3) + thresh);
    }
  
    PID4.Compute();
    if ((PID4.GetMode() == AUTOMATIC && SP4 == 0 && abs(PV4) < 2.0) || (PID4.GetMode() == MANUAL && CV4 == 0)){
      analogWrite(run4, 0);
      analogWrite(reverse4, 0);
    }
    else if(CV4 > 0){
      analogWrite(run4, int(CV4) + thresh);
      analogWrite(reverse4, 0);
    }
    else if(CV4 < 0){
      analogWrite(run4, 0);
      analogWrite(reverse4, int(-CV4) + thresh);
    }
    measured_vel = rpm_2_vel(PV1, PV2, PV3, PV4);
    chatter_vel.publish( &measured_vel );

    delay(interv);

    digitalWrite(13, !digitalRead(13));

    check_connection();

    while(millis() < schedule_time){
      ;
    }
    schedule_time += interv;
    
    nh.spinOnce();
    
}
