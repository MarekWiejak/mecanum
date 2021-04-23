#include <ros/ros.h>
#include <tic.hpp>
#include "../include/tic_crane/server.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crane_server");
    ros::NodeHandle nh;
    tic_crane::MotorServer motor;

    tic_crane::MoveToPositionActionServer AS_mtp("move_to_target_pose", nh, motor.getHandlePtr());
    tic_crane::MeasurementActionServer AS_m("measurement", nh);


    ros::spin();
    return 0;
}
