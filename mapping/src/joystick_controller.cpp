#include <memory>
#include "communicator_library/communicator.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

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