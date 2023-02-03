#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <cmath>

using namespace std;

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

// Global Variables
serial::Serial ser;
sensor_msgs::Imu imu;
geometry_msgs::Twist cmd_vel;

bool imu_received = false;
bool twist_received = false;

float dt = 0.001;

int queue_size = 1000;
float publish_rate = 1000;

void Imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    imu = *imu_msg;
    imu_received = true;
}

void Twist_callback(const geometry_msgs::Twist::ConstPtr& twist_msg){
    cmd_vel = *twist_msg;
    twist_received = true;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

int main(int argc, char **argv){
    Quaternion Q = {0, };
    EulerAngles E = {0, };

    float x_dist = 0, y_dist = 0, z_dist = 0;
    float x_rot = 0, y_rot = 0, z_rot = 0;

    nav_msgs::Odometry odom;

    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", queue_size, Imu_callback);
    ros::Subscriber twist_sub = nh.subscribe("/cmd_vel", queue_size, Twist_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", publish_rate);


    tf2::Quaternion q;
    static tf2_ros::TransformBroadcaster tf_br;
    geometry_msgs::TransformStamped transformStamped;

    //transformStamped.header.stamp = ros::Time(0);
    // transformStamped.header.frame_id = "map";
    // odom.header.frame_id = "map";

    transformStamped.header.frame_id = "odom";
    odom.header.frame_id = "odom";

    transformStamped.child_frame_id = "base_footprint";
    odom.child_frame_id = "base_footprint";

    //odom.header.stamp = ros::Time(0);
    ros::Rate loop_rate(publish_rate);

    int cnt = 0;

    while(ros::ok()){
        ros::spinOnce();
        if(imu_received && twist_received){
            // Header 수정
            odom.header.stamp = ros::Time::now();
            odom.header.seq = cnt;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.seq = cnt;

            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = imu.angular_velocity.z;

            x_rot += odom.twist.twist.angular.x * dt;
            y_rot += odom.twist.twist.angular.y * dt;
            z_rot += odom.twist.twist.angular.z * dt;

            // Linear vel Odom
            odom.twist.twist.linear.x = (cmd_vel.linear.x * cos(z_rot) - cmd_vel.linear.y * sin(z_rot))*0.5;
            odom.twist.twist.linear.y = (cmd_vel.linear.x * sin(z_rot) + cmd_vel.linear.y * cos(z_rot))*0.5;
            odom.twist.twist.linear.z = 0;

            x_dist += odom.twist.twist.linear.x* dt;
            y_dist += odom.twist.twist.linear.y* dt;
            z_dist += cmd_vel.linear.z * dt;

            Q = ToQuaternion(x_rot, y_rot, z_rot);

            //cout << x_dist << endl;

            odom.pose.pose.position.x = x_dist;
            odom.pose.pose.position.y = y_dist;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.w = Q.w;
            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = Q.z;

            transformStamped.transform.translation.x = odom.pose.pose.position.x;
            transformStamped.transform.translation.y = odom.pose.pose.position.y;
            transformStamped.transform.translation.z = odom.pose.pose.position.z;

            transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;
            transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
            transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
            transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
        }
        else{
            odom.header.stamp = ros::Time::now();
            odom.header.seq = cnt;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.seq = cnt;

            odom.pose.pose.position.x = 0;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.w = 1;
            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;

            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.linear.z = 0;

            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = 0;

            transformStamped.transform.translation.x = 0;
            transformStamped.transform.translation.y = 0;
            transformStamped.transform.translation.z = 0;

            transformStamped.transform.rotation.w = 1;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
        }
        odom_pub.publish(odom);
        tf_br.sendTransform(transformStamped);
        cnt++;
        loop_rate.sleep();
    }
    return 0;
}
