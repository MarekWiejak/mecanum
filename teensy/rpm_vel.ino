#define zc 1.15785
#define xc 1.12924
#define yc 1.18122
#define lc 0.18
#define wheel_r 0.039

const double k_forward = ((2*3.1415) * wheel_r/4 / 60);
const double x_forward = (k_forward / xc);
const double y_forward = (k_forward / yc);
const double z_forward = (k_forward / zc / lc);

const double k_inverse = (60 / (2*3.1415) / wheel_r);
const double x_inverse = k_inverse * xc;
const double y_inverse = k_inverse * yc;
const double z_inverse = k_inverse * zc * lc;


geometry_msgs::Twist rpm_2_vel(double rpm1, double rpm2, double rpm3, double rpm4){
  geometry_msgs::Twist vel;
  vel.linear.x = x_forward * (rpm1 + rpm2 + rpm3 + rpm4);
  vel.linear.y = y_forward * (rpm1 - rpm2 + rpm3 - rpm4);
  vel.angular.z = z_forward * (rpm1 - rpm2 - rpm3 + rpm4);
  return  vel;
}

void vel_2_rpm(geometry_msgs::Twist vel){
  SP1 = x_inverse * vel.linear.x + y_inverse * vel.linear.y + z_inverse * vel.angular.z;
  SP2 = x_inverse * vel.linear.x - y_inverse * vel.linear.y - z_inverse * vel.angular.z;
  SP3 = x_inverse * vel.linear.x + y_inverse * vel.linear.y - z_inverse * vel.angular.z;
  SP4 = x_inverse * vel.linear.x - y_inverse * vel.linear.y + z_inverse * vel.angular.z;
}

void check_connection(){
  if (millis() - last_cmd_time > 1000){
    SP1 = 0;
    SP2 = 0;
    SP3 = 0;
    SP4 = 0;
  }
}
