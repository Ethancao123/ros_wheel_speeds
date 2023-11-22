#include "robot_wheel_speeds/KiwiDriveEncoders.h"

KiwiDriveEncoders::KiwiDriveEncoders(
        int leftPinA,
        int leftPinB,
        int rightPinA,
        int rightPinB,
        int rearPinA,
        int rearPinB,
        int ticksPerRevolution,
        int wheelDiameterMm,
        int wheelFrontAxisMm,
        int wheelLengthAxisMm,
        float wheelFrontAngleRad, // 0 is parallel
        const ros::Duration& velocityUpdateInterval
    )
    : m_leftWheel(leftPinA, leftPinB, ticksPerRevolution, wheelDiameterMm, velocityUpdateInterval),
      m_rightWheel(rightPinA, rightPinB, ticksPerRevolution, wheelDiameterMm, velocityUpdateInterval),
      m_rearWheel(rearPinA, rearPinB, ticksPerRevolution, wheelDiameterMm, velocityUpdateInterval),
      m_wheelFrontAxis(float(wheelFrontAxisMm)/1000),
      m_wheelLengthAxis(float(wheelLengthAxisMm)/1000),
      m_wheelFrontAngleSin(std::sin(wheelFrontAngleRad)),
      m_wheelFrontAngleCos(std::cos(wheelFrontAngleRad)),
      m_keepThreadAlive(false)
{
}

KiwiDriveEncoders::~KiwiDriveEncoders()
{
    m_keepThreadAlive = false;
    m_encoderPollingThread.join();
}

void KiwiDriveEncoders::start()
{

    m_keepThreadAlive = true;
    m_leftWheel.init();
    m_rightWheel.init();
    m_rearWheel.init();
    m_encoderPollingThread = std::thread(
        [this]()
        {
            ROS_INFO("Starting polling thread");
            while(m_keepThreadAlive)
            {
                std::lock_guard<std::mutex> lock(m_mutex);

                m_timeNow = ros::Time::now();
                m_leftWheel.DoReading(m_timeNow);
                m_rightWheel.DoReading(m_timeNow);
                m_rearWheel.DoReading(m_timeNow);
            }
        });
}

void KiwiDriveEncoders::stop() {
    m_leftWheel.end();
    m_rightWheel.end();
    m_rearWheel.end();
}

robot_wheel_speeds::WheelVelocities KiwiDriveEncoders::GetVelocities() const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    robot_wheel_speeds::WheelVelocities msg;

    msg.header.stamp = m_timeNow;

    msg.left = m_leftWheel.GetVelocity();
    msg.right = m_rightWheel.GetVelocity();
    msg.rear = m_rearWheel.GetVelocity();
    msg.velocity = calculateTotalVelocity(msg.left, msg.right, msg.rear);

    ROS_DEBUG("GetVelocities { left:%f, right:%f, rear:%f, lin x: %f, lin y: %f, ang:%f }",
              msg.left, msg.right, msg.rear, msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.angular.z);
    return msg;
}

geometry_msgs::Twist KiwiDriveEncoders::calculateTotalVelocity(float left, float right, float rear) const
{
    geometry_msgs::Twist velocities;
    velocities.linear.x = 0.5 * m_wheelFrontAngleSin * (left - right);
    velocities.linear.y = 0.5 * ((m_wheelFrontAngleCos * (left + right)/2) - rear);
    velocities.angular.z = -2*(right + left + rear)/m_wheelFrontAxis/3;

    return velocities;
}
