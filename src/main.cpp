#include <ros/ros.h>

#include "robot_wheel_speeds/KiwiDriveEncoders.h"

int main(int argc, char *argv[])
{
    // Config
    int velocityUpdateIntervalNs = 2e7;
    ros::Duration velocityUpdateInterval(0, velocityUpdateIntervalNs);
    int leftPinA = 17;
    int leftPinB = 18;
    int rightPinA = 23;
    int rightPinB = 22;
    int rearPinA = 24;
    int rearPinB = 25;
    int ticksPerRevolution = 300;
    int wheelDiameterMm = 26;
    int wheelFrontAxisMm = 200;
    int wheelLengthAxisMm = 200;
    float wheelFrontAngle = 0.5236;


    ros::init(argc, argv, "robot_wheel_speeds");
    // SystemGPIO gpio({leftPinA, leftPinB, rightPinA, rightPinB, rearPinA, rearPinB});

    ros::NodeHandle nodeHandle;

    auto publisher = nodeHandle.advertise<robot_wheel_speeds::WheelVelocities>(
                        "wheel_speeds", 10);

    ros::Rate loop_rate(20);

    ROS_INFO("Starting wheel_speeds");
    KiwiDriveEncoders encoders(leftPinA, leftPinB, rightPinA, rightPinB, rearPinA, rearPinB,
                                       ticksPerRevolution, wheelDiameterMm, wheelFrontAxisMm, wheelLengthAxisMm, wheelFrontAngle,
                                       velocityUpdateInterval);
    encoders.start();
    ROS_INFO("Encoders started");
    int seq = 1;
    while(ros::ok())
    {
        // publish here
        auto msg = encoders.GetVelocities();
        msg.header.seq = seq++;

        publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    encoders.stop();
    return 0;
}
