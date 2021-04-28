// #pragma once
#include <ros/ros.h>
#include <tic.hpp>
#include <actionlib/server/simple_action_server.h>

#include <tic_crane/MoveToPositionAction.h>
#include <tic_crane/MeasurementAction.h>

namespace tic_crane{

    // a class handling Pololu TIC controller - a brigde between ROS and the controller 
    class MotorServer {
        public:
            MotorServer();
            ~MotorServer();
            tic::handle* getHandlePtr();

        private:
            tic::handle openHandle(const char * cDeviceSerialNum = nullptr);
            ros::NodeHandle nh_;
            tic::handle handle_;
            tic::variables vars_;
        };

    // an action server class
    class MoveToPositionActionServer {
        protected:
            tic::handle *th_;
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<tic_crane::MoveToPositionAction> as_;
            std::string action_name_;
            tic_crane::MoveToPositionFeedback feedback_;
            tic_crane::MoveToPositionResult result_;

        public:
            MoveToPositionActionServer(std::string name, ros::NodeHandle nh, tic::handle* th) :
                th_(th),
                nh_(nh),
                as_(nh_, name, boost::bind(&MoveToPositionActionServer::executeCB, this, _1, th_), false),
                action_name_(name)
            {
                as_.start();    
            }

            ~MoveToPositionActionServer(void){}
        
            void executeCB(const tic_crane::MoveToPositionGoalConstPtr &goal, tic::handle *th);
        };

    // an action server class
    class MeasurementActionServer {
        protected:
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<tic_crane::MeasurementAction> as_;
            std::string action_name_;
            tic_crane::MeasurementFeedback feedback_;
            tic_crane::MeasurementResult result_;

        public:
            MeasurementActionServer(std::string name, ros::NodeHandle nh) :
                nh_(nh),
                as_(nh_, name, boost::bind(&MeasurementActionServer::executeCB, this, _1), false),
                action_name_(name)
            {
                as_.start();
            }

            ~MeasurementActionServer(void){}

            void executeCB(const tic_crane::MeasurementGoalConstPtr &goal);
        };
};
