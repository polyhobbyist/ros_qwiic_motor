/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifdef _WIN32
#pragma optimize( "", off )
#else
#include "i2c/i2c.h"
#pragma GCC optimize ("O0")
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

uint8_t kMotorNeutral = 128;

typedef enum : uint8_t
{
   MotorCommand_Enable = 0x70,
   MotorCommand_Drive0 = 0x20,
   //..
   //MotorCommand_DriveX = 0x41,
   
} QwiicMotorCommand;


class MotorSubscriber : public rclcpp::Node
{

  public:
    MotorSubscriber()
    : Node("ros_qwiic_motor")
    {
    }

    void start()
    {
        #ifndef _WIN32
        if ((_i2cFileDescriptor = i2c_open("/dev/i2c-1")) == -1) 
        {
            return;
        }

        _i2cDevice.bus = _i2cFileDescriptor;
        _i2cDevice.addr = 0x58;                 // TODO make configurable
        _i2cDevice.tenbit = 0;
        _i2cDevice.delay = 10;
        _i2cDevice.flags = 0;
        _i2cDevice.page_bytes = 8;
        _i2cDevice.iaddr_bytes = 8;
        #endif

        disable();

        get_parameter_or<float>("wheelSeparation", _wheelSeparation, 50);
        get_parameter_or<float>("wheelRadius", _wheelRadius, 10);        

        _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorSubscriber::cmdVelCallback, this, _1));        
    }

  private:
    void enable()
    {
        command(MotorCommand_Enable, 0x01);
    }

    void disable()
    {
        command(MotorCommand_Enable, 0x00);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        enable();

        double angularComponent = _wheelSeparation / (2.0f * _wheelRadius);   // rads / second
        double linearComponent = 1.0f;// _wheelRadius; // cm / second.

        double speedRight = angularComponent * msg->angular.z + linearComponent * msg->linear.x;
        double speedLeft = angularComponent * msg->angular.z - linearComponent * msg->linear.x;

        motor(0, speedRight);
        motor(1, speedLeft);
    }

    void motor(uint8_t channel, double power)
    {
        // Motor controller does 0 - 255
        int8_t powerLevel = (int8_t)((uint8_t)(std::abs(power) * 255.0) >> 7);   // signed

        if (power < 0)
        {
            command(MotorCommand_Drive0 + channel, kMotorNeutral - powerLevel);
        }
        else
        {
            command(MotorCommand_Drive0 + channel, kMotorNeutral + powerLevel);
        }

    }

    void command(uint8_t command, uint8_t value)
    {
        #ifndef _WIN32
        int ret = i2c_ioctl_write(&_i2cDevice, command, &value, 1);
        if (ret == -1 || (size_t)ret != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "failed to write to motor controller: [%d]", ret);
        }
        #endif
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;

    #ifndef _WIN32
    int _i2cFileDescriptor;
    I2CDevice _i2cDevice;
    #endif

    float _wheelSeparation;
    float _wheelRadius;    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorSubscriber>();

    node->declare_parameter("wheelSeparation");
    node->declare_parameter("wheelRadius");

    node->start();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}