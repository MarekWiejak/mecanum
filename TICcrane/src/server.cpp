#include <ros/ros.h>
#include <tic.hpp>
#include <actionlib/server/simple_action_server.h>
#include <cmath>

#include "../include/tic_crane/server.hpp"
#include <tic_crane/MoveToPositionAction.h>
#include <tic_crane/MeasurementAction.h>

namespace tic_crane{

    MotorServer::MotorServer()
    {
        try{
            handle_ = openHandle();
            vars_ = handle_.get_variables();
        }
        catch (const std::exception & error)
        {
            std::cerr << "Error: " << error.what() << std::endl;
        }
    }

    MotorServer::~MotorServer(){
    }
        
    tic::handle MotorServer::openHandle(const char * cDeviceSerialNum)
    {
        std::vector<tic::device> list = tic::list_connected_devices();
        for (const tic::device & device : list)
        {
            if (cDeviceSerialNum && device.get_serial_number() != cDeviceSerialNum) continue;
            return tic::handle(device);
        }
        throw std::runtime_error("No device found.");
    }

    tic::handle* MotorServer::getHandlePtr(){
        return &handle_;
    }


    void MoveToPositionActionServer::executeCB(const tic_crane::MoveToPositionGoalConstPtr &goal, tic::handle *th){
        tic::variables var_;
       
        // Initialization
        th->energize();
        th->exit_safe_start();
        th->set_target_position(goal->position);
        
        // Execution - waiting untill motor reaches goal position
        do{
            th->reset_command_timeout();
            var_ = th->get_variables();
            feedback_.position = var_.get_current_position();
            as_.publishFeedback(feedback_);
        }while( feedback_.position != goal->position );

        // Deenergizing motor after task is completed
        th->deenergize();

        // Sending status and outcome
        result_.position = feedback_.position;
        as_.setSucceeded(result_);
        return;
    }

    // a placeholder for the real measurement action, it does nothing
    void MeasurementActionServer::executeCB(const tic_crane::MeasurementGoalConstPtr &goal){
        ROS_INFO("Starting measurement");
        ROS_INFO("Measurement will take %d seconds", goal->measurement_duration);

        for(int i = 0; i <= goal->measurement_duration; ++i){
            int time_left = goal->measurement_duration - i;
            ROS_INFO("Measurement progress: [ %d / %d ] -- time left: %d seconds", i, goal->measurement_duration, time_left);
            sleep(1);
        }
        ROS_INFO("Measurement completed");

        result_.if_succeded = true;
        as_.setSucceeded(result_);
        return;
    }
    
};
