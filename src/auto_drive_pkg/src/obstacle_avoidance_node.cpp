#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance() : Node("obstacle_avoidance")
    {
        // 声明参数
        this->declare_parameter("safe_distance", 0.5);
        this->declare_parameter("min_front_angle", -30.0);
        this->declare_parameter("max_front_angle", 30.0);
        this->declare_parameter("default_speed", 0.5);
        this->declare_parameter("stop_speed", 0.0);
        this->declare_parameter("turn_angle", 30.0);

        // 获取参数
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        min_front_angle_ = this->get_parameter("min_front_angle").as_double();
        max_front_angle_ = this->get_parameter("max_front_angle").as_double();
        default_speed_ = this->get_parameter("default_speed").as_double();
        stop_speed_ = this->get_parameter("stop_speed").as_double();
        turn_angle_ = this->get_parameter("turn_angle").as_double();

        // 创建订阅者和发布者
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleAvoidance::laserCallback, this, std::placeholders::_1));
        
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_speed", 10);
        servo_pub_ = this->create_publisher<std_msgs::msg::Float32>("steering_angle", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node initialized");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        bool obstacle_detected = false;
        float min_distance = std::numeric_limits<float>::max();
        
        float angle_increment = scan->angle_increment * 180.0 / M_PI;
        float current_angle = scan->angle_min * 180.0 / M_PI;

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            if (current_angle >= min_front_angle_ && current_angle <= max_front_angle_) {
                float range = scan->ranges[i];
                if (range > scan->range_min && range < scan->range_max && range < safe_distance_) {
                    obstacle_detected = true;
                    min_distance = std::min(min_distance, range);
                }
            }
            current_angle += angle_increment;
        }

        auto motor_msg = std_msgs::msg::Float32();
        auto servo_msg = std_msgs::msg::Float32();

        if (obstacle_detected) {
            motor_msg.data = stop_speed_;
            servo_msg.data = turn_angle_;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected at distance: %.2f m", min_distance);
        } else {
            motor_msg.data = default_speed_;
            servo_msg.data = 0.0;
        }

        motor_pub_->publish(motor_msg);
        servo_pub_->publish(servo_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pub_;

    double safe_distance_;
    double min_front_angle_;
    double max_front_angle_;
    double default_speed_;
    double stop_speed_;
    double turn_angle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}