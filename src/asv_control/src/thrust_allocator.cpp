#include <memory>
#include <cmath>
#include <numbers>
#include <functional>
#include <algorithm>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "std_msgs/msg/float64.hpp"
#include "asv_control/msg/tau.hpp"

using std::abs;
using std::numbers::pi;
using namespace std::chrono_literals;
using std::placeholders::_1;


class ThrustAllocator : public rclcpp::Node {

    public:
        ThrustAllocator() : Node ("thrust_allocator"){
            T << 1, 0, 1, 0, 0, 1, 0, 1, -1.027135, -2.373776, 1.027135, -2.373776;
            T_pinv = T.completeOrthogonalDecomposition().pseudoInverse();

            tau.setZero();
            u.setZero();

            tau_sub = this->create_subscription<asv_control::msg::Tau>(
                "/control/tau", 10, std::bind(&ThrustAllocator::thrust_cb, this, _1)
            );
            right_thrust_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
            left_thrust_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
            right_pos_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 10);
            left_pos_pub = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 10);

            timer = this->create_wall_timer(10ms, std::bind(&ThrustAllocator::allocate_thrust, this));
        }

    private:
        Eigen::Vector4d get_optimal_force_vector(Eigen::Vector3d tau){
            return T_pinv * tau;
        }

        void allocate_thrust(){
            Eigen::Vector4d f = get_optimal_force_vector(this->tau);
            u(0) = std::sqrt(f(0)*f(0) + f(1)*f(1));
            u(1) = std::sqrt(f(2)*f(2) + f(3)*f(3));
            u(2) = std::atan2(f(1), f(0));
            u(3) = std::atan2(f(3), f(2));
            
            // Bound limiting
            if(((abs(u(2)) >= pi / 4.0) && (abs(u(2)) <= 3*pi/4)) ){
                // Dead zone, not controllable
                u(0) = 0;
                u(2) = 0;
            } else if(abs(u(2)) > 3*pi/4){
                u(0) = -u(0);
                u(2) = u(2) - pi * (abs(u(2)) / u(2));
            }

            if(((abs(u(3)) >= pi / 4.0) && (abs(u(3)) <= 3*pi/4)) ){
                // Dead zone, not controllable
                u(1) = 0;
                u(3) = 0;
            } else if(abs(u(3)) > 3*pi/4){
                u(1) = -u(1);
                u(3) = u(3) - pi * (abs(u(3)) / u(3));
            }

            double scale = 1.0;
            // Clamping
            for (int i = 0; i < 2; i++) {
                if (u(i) > Pmax) {
                    scale = std::min(scale, Pmax / u(i));
                } else if (u(i) < -Nmax) {
                    scale = std::min(scale, Nmax / -u(i));
                }
            }
            u(0) *= scale;
            u(1) *= scale;

            auto r_thrust_msg = std_msgs::msg::Float64();
            auto l_thrust_msg = std_msgs::msg::Float64();
            auto r_pos_msg = std_msgs::msg::Float64();
            auto l_pos_msg = std_msgs::msg::Float64();

            l_thrust_msg.data = u(0);
            r_thrust_msg.data = u(1);
            l_pos_msg.data = u(2);
            r_pos_msg.data = u(3);

            left_thrust_pub->publish(l_thrust_msg);
            right_thrust_pub->publish(r_thrust_msg);
            left_pos_pub->publish(l_pos_msg);
            right_pos_pub->publish(r_pos_msg);
        }

        void thrust_cb(const asv_control::msg::Tau &msg){
            tau(0) = msg.tx;
            tau(1) = msg.ty;
            tau(2) = msg.mz;
        }

        Eigen::Matrix<double, 3, 4>  T;
        Eigen::Matrix<double, 4, 3>  T_pinv;
        Eigen::Vector3d tau;
        Eigen::Vector4d u; // [Tx, Ty, Delx, Dely]

        double Pmax = 2353.53;
        double Nmax = 1000.0;

        rclcpp::Subscription<asv_control::msg::Tau>::SharedPtr tau_sub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrustAllocator>());
    rclcpp::shutdown();
    return 0;
}