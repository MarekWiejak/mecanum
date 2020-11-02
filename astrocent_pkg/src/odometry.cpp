#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <astrocent/SetOdometry.h>

class Odometer
{
    private:
    ros::Time current_time;
    ros::Time last_time;
    double x;
    double y;
    double th;
    tf::TransformBroadcaster odom_broadcaster;

    public:
    ros::Publisher odom_pub;
    ros::Publisher my_odom;

    Odometer(){
        double x = 0.0;
        double y = 0.0;
        double th = 0.0;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }

    void callback(const geometry_msgs::Twist& msg){
        this->current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (msg.linear.x * cos(th) - msg.linear.y * sin(th)) * dt;
        double delta_y = (msg.linear.x * sin(th) + msg.linear.y * cos(th)) * dt;
        double delta_th = msg.angular.z * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans);


        geometry_msgs::Vector3 position;
        position.x = x;
        position.y = y;
        position.z = th;
        my_odom.publish(position);
        

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom_pub.publish(odom);

        last_time = current_time;
    }

    bool set_odometry(astrocent::SetOdometry::Request &req, astrocent::SetOdometry::Response &res){
        x = req.x;
        y = req.y;
        th = req.z;
        return true;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    Odometer odometer;
    odometer.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    odometer.my_odom = n.advertise<geometry_msgs::Vector3>("my_odom", 50);
    ros::Subscriber vel_sub = n.subscribe("velPV", 1000, &Odometer::callback, &odometer);
    ros::ServiceServer service = n.advertiseService("set_odometry", &Odometer::set_odometry, &odometer);
    
    ros::spin();
    return 0;
    
}