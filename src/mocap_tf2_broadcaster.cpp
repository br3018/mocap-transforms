// Import C++ headers.
#include <functional>
#include <memory>
#include <sstream>
#include <string>

// Import standard ROS2 C++ header.
#include "rclcpp/rclcpp.hpp"
// Import specific ROS2 headers.
#include "tf2_ros/transform_broadcaster.h"
// Import message types.
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

class MocapTF2Broadcaster : public rclcpp::Node
{
    public:
        MocapTF2Broadcaster()
        : Node("mocap_tf2_broadcaster", rclcpp::NodeOptions().enable_logger_service(true))
        {
            RCLCPP_INFO(this->get_logger(), "Mocap TF2 Broadcaster Node has been started.");

            // Initialize transform broadcaster.
            tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
            // Define lambda function for handling incoming RigidBodies messages.
            auto handle_rigid_bodies = [this](const std::shared_ptr<mocap4r2_msgs::msg::RigidBodies> msg){
                geometry_msgs::msg::TransformStamped t;

                // Process each rigid body in the message using for-each loop.
                for (const auto& rigidbody : msg->rigidbodies) {
                    // Populate the header of the TransformStamped message.
                    t.header.stamp = this->get_clock()->now();
                    t.header.frame_id = "world";
                    t.child_frame_id = rigidbody.rigid_body_name;
                    // Populate the transform data.
                    t.transform.translation.x = rigidbody.position.x;
                    t.transform.translation.y = rigidbody.position.y;
                    t.transform.translation.z = rigidbody.position.z;
                    t.transform.rotation.x = rigidbody.orientation.x;
                    t.transform.rotation.y = rigidbody.orientation.y;
                    t.transform.rotation.z = rigidbody.orientation.z;
                    t.transform.rotation.w = rigidbody.orientation.w;

                    // Broadcast the transform.
                    tf_broadcaster_->sendTransform(t);
                }
            };

            // Subscribe to the rigid_bodies topic. 
            subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
                "rigid_bodies", 10,
                handle_rigid_bodies);
        }

private:
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapTF2Broadcaster>());
  rclcpp::shutdown();
  return 0;
}