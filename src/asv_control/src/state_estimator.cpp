#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "asv_control/msg/body_velocities.hpp"
#include "asv_control/msg/world_relative_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using std::sin;
using std::cos;

class StateEstimator : public rclcpp::Node {

    public:
        StateEstimator() : Node("state_estimator"){
            
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "/wamv/sensors/imu/imu/data", 10, std::bind(&StateEstimator::imu_callback, this, _1)
            );

            pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/model/wamv/odometry", 10, std::bind(&StateEstimator::pose_callback, this, _1)
            );

        }

    private:
        void imu_callback(const sensor_msgs::msg::Imu &msg) {
            RCLCPP_INFO(this->get_logger(), "IMU");
        }

        void pose_callback(const nav_msgs::msg::Odometry &msg) {
            const auto& rots = msg.pose.pose.orientation;
            double roll, pitch, yaw;

            tf2::Quaternion quat(
                rots.x,
                rots.y,
                rots.z,
                rots.w
            );
            tf2::Matrix3x3 matrix(quat);
            matrix.getRPY(roll, pitch, yaw);

            eta << msg.pose.pose.position.x , msg.pose.pose.position.y, yaw;
            v << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z;
        }

        Eigen::Matrix3d J(double theta){
            Eigen::Matrix3d mat {
                {cos(theta), -sin(theta), 0},
                {sin(theta), cos(theta), 0},
                {0, 0, 1}
            };
            return mat;
        }

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
        Eigen::Vector3d eta;
        Eigen::Vector3d eta_dot;
        Eigen::Vector3d v;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimator>());
    rclcpp::shutdown();
    return 0;
}