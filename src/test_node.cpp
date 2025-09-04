#include "rclcpp/rclcpp.hpp"

class TestNode : public rclcpp::Node
{
    public:
    TestNode() : Node("test_node") {
        RCLCPP_INFO(this->get_logger(), "Hello Worlds.");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Node is running.");
            }   
        );
    }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   