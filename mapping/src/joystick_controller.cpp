#include <memory>
// #include "communicator.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

#include </usr/include/libserial/SerialStream.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

class SerialCommunicator {

private:

struct data_packet
{
    int init;
    int deinit;
    int emergency_stop;
    int direction_r;
    int motor_r;
    int direction_l;
    int motor_l;
    int servo;

    std::string toString()
    {
    return std::to_string(init) + " " + std::to_string(deinit)  + " " + std::to_string(emergency_stop)
           + " " + std::to_string(direction_r) + " " + std::to_string(motor_r) 
           + " " + std::to_string(direction_l) + " " + std::to_string(motor_l)
           + " " + std::to_string(servo);
    }
}; 

LibSerial::SerialStream serial;

public:
        SerialCommunicator() {}
        data_packet dp = {};

        void init_comms(const std::string& port) {
            serial.Open(port);
            serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial.SetParity(LibSerial::Parity::PARITY_NONE);
            serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        }

        void init_motors() {
            
            dp.init = 1;
            dp.deinit = 0;
            dp.emergency_stop = 0;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = 0;

            serial << dp.toString() << std::endl;
        }

        void deinit_motors(){
            dp.init = 0;
            dp.deinit = 1;
            dp.emergency_stop = 0;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = 0;

            serial << dp.toString() << std::endl;
        }

        void send_command_motors(int direction_r, int speed_r, int direction_l, int speed_l, int servo_pwm) {
            dp.init = 1;
            dp.deinit = 0;
            dp.emergency_stop = 0;
            dp.direction_r = direction_r;
            dp.motor_r = speed_r;
            dp.direction_l = direction_l;
            dp.motor_l = speed_l;
            dp.servo = servo_pwm;
            serial << dp.toString() << std::endl;
        }

        void read() {
           const int BUFFER_SIZE = 256;
            char input_buffer[BUFFER_SIZE];
            serial.read( input_buffer, BUFFER_SIZE);
        }

        void close() {
            serial.Close();
        }
};

class MinimalSubscriber : public rclcpp::Node
{
  public:

    SerialCommunicator communicator;
    

    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      communicator.init_comms("/dev/ttyACM0");
      communicator.init_motors();
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy & msg) 
    {
      int forward_backward = (int)msg.axes[1] * 65535;

      if(forward_backward > 0)
      {
        communicator.send_command_motors(0, forward_backward, 0, forward_backward, 0);
      }
      else if(forward_backward < 0)
      {
        communicator.send_command_motors(1, forward_backward, 1, forward_backward, 0);        
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}