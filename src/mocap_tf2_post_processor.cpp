// ROS2 Mocap TF2 Post Processor Node.
// This node listens for GRASP and RAFTI transforms and publishes the relative pose and twist

// Import C++ headers.
#include <functional>
#include <chrono>
#include <memory>
#include <string>

// Import standard ROS2 C++ header.
#include "rclcpp/rclcpp.hpp"
// Import specific ROS2 headers.
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
// Import message types.
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Define constants. 
const int GRASP_STREAM_ID = 1;
const int RAFTI_STREAM_ID = 2;
const float AVERAGING_INTERVAL = 0.1; // [s]
const float PROCESSING_FREQUENCY = 10.0; // [Hz]

class MocapTF2PostProcessor : public rclcpp::Node
{
public:
    MocapTF2PostProcessor()
    : Node("mocap_tf2_post_processor")
    {
        RCLCPP_INFO(this->get_logger(), "Mocap TF2 Post Processor Node has been started.");

        // Initialize TF2 buffer and listener.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create relative pose and twist publisher GRASP w.r.t RAFTI.
        relative_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("relative_pose", 1);
        relative_twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("relative_twist", 1);

        // Create timer for periodic processing.
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0 / PROCESSING_FREQUENCY),
            std::bind(&MocapTF2PostProcessor::process_transforms, this));

    }

private:
    // TF2 buffer and listener.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relative_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr relative_twist_publisher_;

    // Timer.
    rclcpp::TimerBase::SharedPtr timer_;

    // Process transforms.
    void process_transforms()
    {
        // Get the latest transforms.
        geometry_msgs::msg::TransformStamped relative_transform;

        try
        {
            relative_transform = tf_buffer_->lookupTransform(std::to_string(RAFTI_STREAM_ID), std::to_string(GRASP_STREAM_ID), tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        // Convert transforms to pose messages.
        geometry_msgs::msg::PoseStamped relative_pose;
        relative_pose.header.stamp = rclcpp::Clock().now();
        relative_pose.header.frame_id = std::to_string(RAFTI_STREAM_ID);
        relative_pose.pose.position.x = relative_transform.transform.translation.x;
        relative_pose.pose.position.y = relative_transform.transform.translation.y;
        relative_pose.pose.position.z = relative_transform.transform.translation.z;
        relative_pose.pose.orientation = relative_transform.transform.rotation;

        

        // Publish the relative pose.
        relative_pose_publisher_->publish(relative_pose);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapTF2PostProcessor>());
  rclcpp::shutdown();
  return 0;
}