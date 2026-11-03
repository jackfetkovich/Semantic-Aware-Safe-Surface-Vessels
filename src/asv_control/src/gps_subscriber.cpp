#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
using std::placeholders::_1;

class GPSSubscriber : public rclcpp::Node
{
  public:
    GPSSubscriber()
    : Node("gps_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/wamv/sensors/gps/gps/fix", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::NavSatFix & msg) const
    {
        double lat = msg.longitude;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", lat);
    }
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSSubscriber>());
  rclcpp::shutdown();
  return 0;
}