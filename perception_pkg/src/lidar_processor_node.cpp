#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

class LidarProcessor: public rclcpp::Node
{
public:
    LidarProcessor(): Node("lidar_processor")
    {
        subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1)
        );
        
        publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("/obstacle_data", 10);
        
        RCLCPP_INFO(get_logger(), "LİDAR PROCESSOR READY");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    float get_min_distance(const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                          float center_angle, float fov_radians)
    {
        float min_dist = std::numeric_limits<float>::infinity();
        int valid_count = 0;
        
        float half_fov = fov_radians / 2.0;
        float start_angle = center_angle - half_fov;
        float end_angle = center_angle + half_fov;
        
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float current_angle = scan->angle_min + i * scan->angle_increment;
            
            while (current_angle > M_PI) current_angle -= 2.0 * M_PI;
            while (current_angle < -M_PI) current_angle += 2.0 * M_PI;
            
            bool in_range = false;
            
            if (start_angle <= end_angle) {
                in_range = (current_angle >= start_angle && current_angle <= end_angle);
            }
            else {
                in_range = (current_angle >= start_angle || current_angle <= end_angle);
            }
            
            if (in_range) {
                float dist = scan->ranges[i];
                
                if (std::isfinite(dist) && dist >= scan->range_min && dist <= scan->range_max) {
                    min_dist = std::min(min_dist, dist);
                    valid_count++;
                }
            }
        }
        
        if (valid_count == 0 || std::isinf(min_dist)) {
            return scan->range_max;
        }
        
        return min_dist;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        
        float front_angle = 0.0;           
        float right_angle = -M_PI / 2.0;   
        float left_angle = M_PI / 2.0;     
        float back_angle = M_PI;           
        
        float fov = 20.0 * M_PI / 180.0; 
        
        float front_dist = get_min_distance(scan, front_angle, fov);
        float right_dist = get_min_distance(scan, right_angle, fov);
        float left_dist = get_min_distance(scan, left_angle, fov);
        float back_dist = get_min_distance(scan, back_angle, fov);
        
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {front_dist, right_dist, left_dist, back_dist};
        
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
