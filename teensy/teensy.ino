#include <ros.h>
#include <astrocent/Rpm.h>
#include <astrocent/RpmPwm.h>
#include <astrocent/PIDsettings.h>
#include <astrocent/ManualPWM.h>
#include <Encoder.h>
#include <PID_v1.h>

ros::NodeHandle nh;
astrocent::RpmPwm publishPVCV;

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
const long interval = 25; //main loop interval
const float st = 0.5; //settling time (control algorithm)
const int thresh = 30; // threshold of pwm value for motor to start turning
const double count2rpm = interval*pulses_per_rotation/60/1000;

Encoder encoder1(encoder1Pin1, encoder1Pin2);
Encoder encoder2(encoder2Pin1, encoder2Pin2);
Encoder encoder3(encoder3Pin1, encoder3Pin2);
Encoder encoder4(encoder4Pin1, encoder4Pin2);

// PID settings
const double Kp_1 = 1.039/st, Ki_1 = Kp_1/0.2, Kd_1 = 0;
const double Kp_2 = 0.988/st, Ki_2 = Kp_2/0.2, Kd_2 = 0;
const double Kp_3 = 1.233/st, Ki_3 = Kp_3/0.18, Kd_3 = 0;
const double Kp_4 = 1.041/st, Ki_4 = Kp_4/0.19, Kd_4 = 0;

double CV1 = 0, CV2 = 0, CV3 = 0, CV4 = 0;
double PV1 = 0, PV2 = 0, PV3 = 0, PV4 = 0;
double SP1 = 0, SP2 = 0, SP3 = 0, SP4 = 0;

// initialize PIDs
PID PID1(&PV1, &CV1, &SP1, Kp_1, Ki_1, Kd_1, DIRECT);
PID PID2(&PV2, &CV2, &SP2, Kp_2, Ki_2, Kd_2, DIRECT);
PID PID3(&PV3, &CV3, &SP3, Kp_3, Ki_3, Kd_3, DIRECT);
PID PID4(&PV4, &CV4, &SP4, Kp_4, Ki_4, Kd_4, DIRECT);

void messageRPM(const astrocent::Rpm& rpmSP){
  SP1 = rpmSP.rpm1;
  SP2 = rpmSP.rpm2;
  SP3 = rpmSP.rpm3;
  SP4 = rpmSP.rpm4;
}

void messageSettings(const astrocent::PIDsettings& settings){
  if (settings.mode == 0){
    PID1.SetMode(MANUAL);
    PID2.SetMode(MANUAL);
    PID3.SetMode(MANUAL);
    PID4.SetMode(MANUAL);
  }
  else if (settings.mode == 1){
    PID1.SetMode(AUTOMATIC);
    PID2.SetMode(AUTOMATIC);
    PID3.SetMode(AUTOMATIC);
    PID4.SetMode(AUTOMATIC);
  }
}

void messageManPWM(const astrocent::ManualPWM& manPWM){
    digitalWrite(13, LOW);
  if (PID1.GetMode() == MANUAL)
    CV1 = manPWM.pwm1;
  if (PID2.GetMode() == MANUAL)
    CV2 = manPWM.pwm2;
  if (PID3.GetMode() == MANUAL)
    CV3 = manPWM.pwm3;
  if (PID4.GetMode() == MANUAL)
    CV4 = manPWM.pwm4;
}

ros::Publisher chatter("rpmPV", &publishPVCV);
ros::Subscriber<astrocent::Rpm> sub_rpm("rpmSP", &messageRPM);
ros::Subscriber<astrocent::PIDsettings> sub_PIDsettings("driverPIDsettings", &messageSettings);
ros::Subscriber<astrocent::ManualPWM> sub_manPWM("manPWM", &messageManPWM);

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
  nh.advertise(chatter);
  nh.subscribe(sub_rpm);
  nh.subscribe(sub_PIDsettings);
  nh.subscribe(sub_manPWM);

  PID1.SetSampleTime(interval);
  PID2.SetSampleTime(interval);
  PID3.SetSampleTime(interval);
  PID4.SetSampleTime(interval);

  PID1.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID2.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID3.SetOutputLimits(-255 + thresh, 255 - thresh);
  PID4.SetOutputLimits(-255 + thresh, 255 - thresh);
  
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  PID3.SetMode(AUTOMATIC);
  PID4.SetMode(AUTOMATIC);
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
    if(CV1 > 0){
      analogWrite(run1, int(CV1) + thresh);
      analogWrite(reverse1, 0);
    }
    else if(CV1 < 0){
      analogWrite(run1, 0);
      analogWrite(reverse1, int(-CV1) + thresh);
    }
    else if(CV1 == 0){
      analogWrite(run1, 0);
      analogWrite(reverse1, 0);
    }

    PID2.Compute();
    if(CV2 > 0){
      analogWrite(run2, int(CV2) + thresh);
      analogWrite(reverse2, 0);
    }
    else if(CV2 < 0){
      analogWrite(run2, 0);
      analogWrite(reverse2, int(-CV2) + thresh);
    }
    else if(CV2 == 0){
      analogWrite(run2, 0);
      analogWrite(reverse2, 0);
    }
    
    PID3.Compute();
    if(CV3 > 0){
      analogWrite(run3, int(CV3) + thresh);
      analogWrite(reverse3, 0);
    }
    else if(CV3 < 0){
      analogWrite(run3, 0);
      analogWrite(reverse3, int(-CV3) + thresh);
    }
    else if(CV3 == 0){
      analogWrite(run3, 0);
      analogWrite(reverse3, 0);
    }

    PID4.Compute();
    if(CV4 > 0){
      analogWrite(run4, int(CV4) + thresh);
      analogWrite(reverse4, 0);
    }
    else if(CV4 < 0){
      analogWrite(run4, 0);
      analogWrite(reverse4, int(-CV4) + thresh);
    }
    else if(CV4 == 0){
      analogWrite(run4, 0);
      analogWrite(reverse4, 0);
    }

    publishPVCV.rpm1 = PV1;
    publishPVCV.rpm2 = PV2;
    publishPVCV.rpm3 = PV3;
    publishPVCV.rpm4 = PV4;

    publishPVCV.pwm1 = CV1;
    publishPVCV.pwm2 = CV2;
    publishPVCV.pwm3 = CV3;
    publishPVCV.pwm4 = CV4;
    
    chatter.publish( &publishPVCV );
    delay(interval);

      digitalWrite(13, !digitalRead(13));
    
    nh.spinOnce();
    
}
