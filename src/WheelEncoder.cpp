#include "robot_wheel_speeds/WheelEncoder.h"

#include <cmath>
#include <gpiod.h>
#include <unistd.h>
#include <ros/console.h>

WheelEncoder::WheelEncoder(int pinA,
                           int pinB,
                           unsigned int ticksPerRevolution,
                           unsigned int wheelDiameter,
                           const ros::Duration& updateInterval)
    : m_pinA(pinA),
      m_pinB(pinB),
      m_ticksPerRevolution(ticksPerRevolution),
      m_wheelCircumference(2 * M_PI * wheelDiameter),
      m_velocityUpdateInterval(updateInterval),
      m_prevValueA(0),
      m_lastUpdateTime(ros::Time(0)),
      m_direction(0),
      m_ticks(0),
      m_velocity(0),
      chip(NULL),
      lineA(NULL),
      lineB(NULL)
{
    
}

void WheelEncoder::init() {

    chip = gpiod_chip_open_lookup("/dev/gpiochip0");
    if(chip == NULL)
        ROS_ERROR("No GPIO Chip Found");
    lineA = gpiod_chip_get_line(chip, m_pinA);
    lineB = gpiod_chip_get_line(chip, m_pinB);
    gpiod_line_request_input(lineA, "drive");
    gpiod_line_request_input(lineB, "drive");
}

void WheelEncoder::end() {
    gpiod_line_release(lineA);
    gpiod_line_release(lineB);
    gpiod_chip_close(chip);
}   

void WheelEncoder::DoReading(const ros::Time& timenow)
{
    int valueA = gpiod_line_get_value(lineA);
    int valueB = gpiod_line_get_value(lineB);

    if (m_prevValueA == 0 && valueA == 1) // rising edge
    {
        m_direction = (valueB == 1) ? 1 : -1;
        m_ticks++;
    }
    MaybeUpdateVelocity(timenow);
    m_prevValueA = valueA;
}

void WheelEncoder::MaybeUpdateVelocity(const ros::Time& timenow)
{
    if (m_lastUpdateTime.isZero())
    {
        m_lastUpdateTime = timenow;
        return;
    }
    auto dt = timenow - m_lastUpdateTime;

    if (dt > m_velocityUpdateInterval)
    {
        UpdateVelocity(dt);
        m_lastUpdateTime = timenow;
    }
}


void WheelEncoder::UpdateVelocity(const ros::Duration& dt)
{
    float distance = float(m_ticks)/m_ticksPerRevolution * (m_wheelCircumference/1000.0); //mm to m
    m_velocity = distance / dt.toSec() * m_direction;
    m_ticks = 0;
}
