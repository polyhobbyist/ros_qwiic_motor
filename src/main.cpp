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
   MotorCommand_Id = 0x01,
   MotorCommand_Status = 0x77,
   MotorCommand_Enable = 0x70,
   MotorCommand_Drive0 = 0x20,
   //..
   //MotorCommand_DriveX = 0x41,
   
} QwiicMotorCommand;

typedef enum : uint8_t
{
    MotorStatusBit_Enum = 0x01,
    MotorStatusBit_Busy = 0x02,
    MotorStatusBit_Read = 0x04,
    MotorStatusBit_Write = 0x08,
    MotorStatusBit_Enable = 0x10

} QwiicMotorStatusBit;


class MotorSubscriber : public rclcpp::Node
{

  public:
    MotorSubscriber()
    : Node("ros_qwiic_motor")
    , _wheelSeparation(0.1)
    , _wheelRadius(0.03)
    , _powerScale(1.0) // power -> RPM
    , _leftInvert(false)
    , _rightInvert(false)
    , _id(0x5D)

    {
    }

    void start()
    {
        #ifndef _WIN32
        get_parameter_or<uint8_t>("id", _id, 0x5D);        

        if ((_i2cFileDescriptor = i2c_open("/dev/i2c-1")) == -1) 
        {
            return;
        }

        i2c_init_device(&_i2cDevice);

        _i2cDevice.bus = _i2cFileDescriptor;
        _i2cDevice.addr = _id;
        #endif

        rclcpp::Rate loop_rate(1);
        uint8_t id = getId();
        RCLCPP_INFO(rclcpp::get_logger("motor"), "Communicating with motor id: [%d]", id);

        while (!ready())
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "Waiting for Motor Controller to be ready");  
            loop_rate.sleep();
        }

        while (busy())
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "Waiting for Motor Controller, to not be busy...");  
            loop_rate.sleep();
        }
        
        enable();


        get_parameter_or<float>("wheelSeparation", _wheelSeparation, .10);
        get_parameter_or<float>("wheelRadius", _wheelRadius, 0.03);        
        get_parameter_or<float>("powerScale", _powerScale, 1.0);        
        get_parameter_or<bool>("leftInverted", _leftInvert, false);        
        get_parameter_or<bool>("rightInverted", _rightInvert, false);        

        _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorSubscriber::cmdVelCallback, this, _1));        
    }

  private:
    uint8_t getId()
    {
        uint8_t id = 0;
        int ret = i2c_read(&_i2cDevice, MotorCommand_Id, &id, 1);
        if (ret == -1 || (size_t)ret != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "failed to read motor id: [%d]", ret);
        }

        return id;
    }

    // interesting that ready != busy
    bool ready()
    {
        uint8_t status = 0;
        int ret = i2c_read(&_i2cDevice, MotorCommand_Status, &status, 1);
        if (ret == -1 || (size_t)ret != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "failed to read motor status: [%d]", ret);
        }

        if (status != 0xFF &&
            (status & MotorStatusBit_Enum) == MotorStatusBit_Enum)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool busy()
    {
        uint8_t status = 0;
        int ret = i2c_read(&_i2cDevice, MotorCommand_Status, &status, 1);
        if (ret == -1 || (size_t)ret != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("motor"), "failed to read motor status: [%d]", ret);
        }

        if (status & (MotorStatusBit_Busy | MotorStatusBit_Read | MotorStatusBit_Write))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

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
        double speedRight = msg->linear.x - msg->angular.z;
        double speedLeft = msg->linear.x + msg->angular.z;

        motor(0, (_rightInvert? -1.0 : 1.0) * (speedRight * (_wheelSeparation / 2.0) / _wheelRadius) * _powerScale);
        motor(1, (_leftInvert? -1.0 : 1.0) * (speedLeft *(_wheelSeparation / 2.0) / _wheelRadius) * _powerScale);
    }

    void motor(uint8_t channel, double power)
    {
        double powerAdjustment = std::abs(power);
        if (powerAdjustment > 1.0)
        {
            powerAdjustment = 1.0;
        }

        // Motor controller does 0 +/- 255, with 128 as median
        uint8_t powerLevel = (uint8_t)(powerAdjustment * 127.0);

        if (power < 0.0)
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
    float _powerScale;
    bool _leftInvert;
    bool _rightInvert;
    uint8_t _id;
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