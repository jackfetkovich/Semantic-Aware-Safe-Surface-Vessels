#include <memory>
#include <chrono>
#include <Eigen/Dense>
#include "asv_control/msg/world_relative_state.hpp"
#include "asv_control/msg/goal_waypoint.msg"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PIDController : public rclcpp::Node {

    public:
        PIDController : Node ("pid_controller"){
            state_sub = this->create_subscription<asv_control::msg::WorldRelativeState>(
                "/state/worldrelative", 10, std::bind(&StateEstimator::state_callback, this, _1)
            );
            timer = this->create_wall_timer(10ms, std::bind(&PIDController::control_loop, this));
        }



    private:

        void control_loop(){
            double tau = controller();
        }

        double controller(){
            Eigen::Vector3d error = goal - state.head(3);
            error(3) = std::atan2(
                std::sin(error(3)),
                std::cos(error(3))
            );
            
            double tau = kp * error;
            last_error = error;

            return tau;
        }

        void state_callback(const asv_control::msg::WorldRelativeState &msg){
            state(0) = msg.x;
            state(1) = msg.y;
            state(2) = msg.theta;
            state(3) = msg.x_dot;
            state(4) = msg.y_dot;
            state(5) = msg.theta_dot;
        }

        rclcpp::Subscription<asv_control::msg::WorldRelativeState>::SharedPtr state_sub;
        Eigen::Matrix3d kp = Eigen::Matrix::Zero();
        Eigen::Matrix3d kd; = Eigen::Matrix::Zero();
        Eigen::Vector3d goal;
        Eigen::Vector6d state;
        Eigen::Vector3d last_error;
        double last_control_time = 0.0;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}