#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class TfPublisher : public rclcpp::Node
{
public:
    TfPublisher()
    : Node("tf_publisher")
    {
        tf_broadcaster_1 = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_2 = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_3 = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_transforms();
        RCLCPP_INFO(this->get_logger(), "Transforms have been published.");
    }

private:
    void publish_transforms()
    {
        auto now = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped t1;
        t1.header.stamp = now;
        t1.header.frame_id = "base_link";
        t1.child_frame_id = "laser";
        t1.transform.translation.x = 0.24;
        t1.transform.translation.y = 0.0;
        t1.transform.translation.z = 0.1;
        t1.transform.rotation.x = 0.0;
        t1.transform.rotation.y = 0.0;
        t1.transform.rotation.z = 0.0;
        t1.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = now;
        t2.header.frame_id = "base_footprint";
        t2.child_frame_id = "base_link";
        t2.transform.translation.x = 0.0;
        t2.transform.translation.y = 0.0;
        t2.transform.translation.z = 0.0;
        t2.transform.rotation.x = 0.0;
        t2.transform.rotation.y = 0.0;
        t2.transform.rotation.z = 0.0;
        t2.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped t3;
        t3.header.stamp = now;
        t3.header.frame_id = "body";
        t3.child_frame_id = "base_footprint";
        t3.transform.translation.x = 0.0;
        t3.transform.translation.y = 0.0;
        t3.transform.translation.z = 0.0;
        t3.transform.rotation.x = 0.0;
        t3.transform.rotation.y = 0.0;
        t3.transform.rotation.z = 0.0;
        t3.transform.rotation.w = 1.0;

        tf_broadcaster_1->sendTransform(t1);
        tf_broadcaster_2->sendTransform(t2);
        tf_broadcaster_3->sendTransform(t3);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_1;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_2;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_3;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}