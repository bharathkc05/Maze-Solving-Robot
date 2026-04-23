#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class WallFollower: public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obstacle_data", 10,
            std::bind(&WallFollower::obstacle_callback, this, std::placeholders::_1)
        );

        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",10,
            std::bind(&WallFollower::odom_callback,this,std::placeholders::_1)
        );

        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WallFollower::control_loop, this)
        );

        current_state = State::GO;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    enum class State {GO,TURN_RIGHT,TURN_LEFT,STOP};	
	State current_state;

    // control
    float front = 10.0, right = 10.0, left = 10.0, back = 10.0;
    bool is_start = false;
    float min_front_distance = 0.45;
    float min_side_difference = 0.20;
    double desired_yaw;

    // odom 
    double start_x = 0.0, start_y = 0.0, start_yaw = 0.0;
	double current_x = 0.0, current_y = 0.0, current_yaw = 0.0;


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
	{
			current_x = odom->pose.pose.position.x;
			current_y = odom->pose.pose.position.y;

			tf2::Quaternion q;
			tf2::fromMsg(odom->pose.pose.orientation,q);
			tf2::Matrix3x3 m(q);
			double roll,pitch,yaw;
			m.getRPY(roll,pitch,yaw);

			current_yaw = yaw;
			if(!is_start){
				start_x = current_x;
				start_y = current_y;
				start_yaw = current_yaw;
				is_start = true;
			}
	}

    void obstacle_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
            front = msg->data[0];
            right = msg->data[1];
            left  = msg->data[2];
            back  = msg->data[3];

            if(!is_start)
                is_start = true;
    }

    void correct_heading(geometry_msgs::msg::Twist& velocity) {
        double current_deg = current_yaw * 180.0 / M_PI;
        double nearest_90 = round(current_deg / 90.0) * 90.0;

        double error_deg = nearest_90 - current_deg;

        if (fabs(error_deg) > 2.0) {
            double error_rad = error_deg * M_PI / 180.0;

            double correction = std::max(-0.15, std::min(0.15, error_rad * 0.3));
            velocity.angular.z = correction;
        }
    }


    void control_loop(){

        std::string state_str = (current_state == State::GO) ? "GO" : 
                               (current_state == State::TURN_RIGHT) ? "TURN_RIGHT" : 
                               (current_state == State::TURN_LEFT) ? "TURN_LEFT" : "STOP";

        auto velocity = geometry_msgs::msg::Twist();

        if(current_state == State::GO){
            double distance_traveled = sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));

            if (distance_traveled >= 0.3) {
                if ((right - left) > min_side_difference && front > min_front_distance){
                    RCLCPP_INFO(get_logger(),"TURNING RIGHT! Distance: %.2f, Front: %.2f, Right: %.2f, Left: %.2f, Diff: %.2f", 
                                distance_traveled, front, right, left, (right - left));
                    current_state = State::TURN_RIGHT;
                    start_yaw = current_yaw;

                    RCLCPP_INFO(get_logger(),"turn right-1, start_yaw: %f", start_yaw);
                }
                else if (left > right && front <= min_front_distance){
                    current_state = State::TURN_LEFT;
                    start_yaw = current_yaw;
                    RCLCPP_INFO(get_logger(),"turn left-1, start_yaw: %f", start_yaw);
                }
            }

            if(front > min_front_distance){
                velocity.linear.x = 0.2;
                velocity.linear.y = 0.0;
                correct_heading(velocity);
                RCLCPP_INFO(get_logger(),"go-1");
            }
        }
        else if (current_state == State::TURN_LEFT){
            double delta_yaw = current_yaw - start_yaw;

            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

            if (delta_yaw < (M_PI / 2.0)) {
                velocity.linear.x = 0.0;
                velocity.angular.z = 0.25; 
            }
            else {
                current_state = State::GO;
                RCLCPP_INFO(get_logger(),"go-2");
                start_x = current_x;
                start_y = current_y;
            }
        }
        else if (current_state == State::TURN_RIGHT)
        {
            double delta_yaw = current_yaw - start_yaw;

            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

            if (delta_yaw > -(M_PI / 2.0)) {
                velocity.linear.x = 0.0;
                velocity.angular.z = -0.25; 
            }
            else {
                current_state = State::GO;
                RCLCPP_INFO(get_logger(),"go-3");
                start_x = current_x;
                start_y = current_y;
            }
        }

        publisher_->publish(velocity);

    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
