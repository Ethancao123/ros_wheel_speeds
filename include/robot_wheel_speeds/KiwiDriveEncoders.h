#pragma once

#include "robot_wheel_speeds/WheelEncoder.h"
#include "robot_wheel_speeds/WheelVelocities.h"

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <thread>
#include <atomic>
#include <mutex>

class KiwiDriveEncoders
{
public:
    KiwiDriveEncoders(int leftPinA, int leftPinB, int rightPinA, int rightPinB, int rearPinA, int rearPinB,
                               int ticksPerRevolution, int wheelDiameterMm, int wheelFrontAxisMm, int wheelLengthAxisMm, float wheelFrontAngleRad,
                               const ros::Duration& velocityUpdateInterval);

~KiwiDriveEncoders();

    void start();
    void stop();
    robot_wheel_speeds::WheelVelocities GetVelocities() const;

private:
    geometry_msgs::Twist calculateTotalVelocity(float left, float right, float rear) const;

    WheelEncoder m_leftWheel;
    WheelEncoder m_rightWheel;
    WheelEncoder m_rearWheel;
    const float  m_wheelFrontAxis; // spacing between wheels in metres
    const float  m_wheelLengthAxis;
    const float  m_wheelFrontAngleSin;
    const float  m_wheelFrontAngleCos;

    ros::Time          m_timeNow;
    std::thread        m_encoderPollingThread;
    std::atomic_bool   m_keepThreadAlive;
    mutable std::mutex m_mutex;
};
