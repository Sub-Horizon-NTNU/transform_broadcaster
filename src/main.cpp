#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "TransformBroadcasterNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto transform_broadcaster_node = std::make_shared<TransformBroadcasterNode>();
    transform_broadcaster_node->init();
    rclcpp::spin(transform_broadcaster_node);
    rclcpp::shutdown();
    return 0;
}
