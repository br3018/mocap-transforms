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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Import message types.
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

// Define constants.
const int GRASP_STREAM_ID = 1; // Streaming ID for GRASP
const int RAFTI_STREAM_ID = 2; // Streaming ID for RAFTI
const float PROCESSING_FREQUENCY = 50.0; // [Hz] Frequency to update pose and twist
const auto TWIST_COMPUTE_INTERVAL = std::chrono::milliseconds(100); // [ms] (100 ms) Interval to compute twist
const double MIN_DT = 1e-4; // [s] Minimum allowed time difference in seconds between transforms for twist computation

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
        this->create_publisher<geometry_msgs::msg::PoseStamped>("relative_pose", 10);
        relative_twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("relative_twist", 10);

        // while(!initialised_tf){
        //     try{
        //         prev_relative_transform = tf_buffer_->lookupTransform(std::to_string(RAFTI_STREAM_ID), std::to_string(GRASP_STREAM_ID), tf2::TimePointZero);
        //         // Convert to tf::Transform
        //         tf2::fromMsg(prev_relative_transform.transform, prev_relative_tf);
        //         prev_relative_transform.header.stamp = this->get_clock()->now();
        //         initialised_tf = true;
        //     }
        //     catch (tf2::TransformException &ex)
        //     {
        //         RCLCPP_WARN(this->get_logger(), "Initial transform error: %s", ex.what());
        //     }
        // }

        prev_relative_transform.header.stamp = this->get_clock()->now();

        subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
                "rigid_bodies", 10,
                std::bind(&MocapTF2Broadcaster::cb_rigid_bodies, this, _1));

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

    // Subscribers.
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_subscription_;

    // Timer.
    rclcpp::TimerBase::SharedPtr timer_;

    // store for last transforms
    tf2::Transform prev_relative_tf;
    geometry_msgs::msg::TransformStamped prev_relative_transform;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> current_subscribed_relative{nullptr};

    // bool value to initialise last transform value
    bool initialised_tf{false};

    void cb_rigid_bodies(const std::shared_ptr<mocap4r2_msgs::msg::RigidBodies> msg){
        geometry_msgs::msg::TransformStamped rafti;
        geometry_msgs::msg::TransformStamped grasp;
        for(const auto& rigidbody : last_rigid_bodies->rigidbodies) {
            geometry_msgs::msg::TransformStamped t;
            // Populate the header of the TransformStamped message.
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = rigidbody.rigid_body_name;
            // Populate the transform data.
            t.transform.translation.x = rigidbody.pose.position.x;
            t.transform.translation.y = rigidbody.pose.position.y;
            t.transform.translation.z = rigidbody.pose.position.z;
            t.transform.rotation.x = rigidbody.pose.orientation.x;
            t.transform.rotation.y = rigidbody.pose.orientation.y;
            t.transform.rotation.z = rigidbody.pose.orientation.z;
            t.transform.rotation.w = rigidbody.pose.orientation.w;

            if(rigidbody.rigid_body_name == std::to_string(RAFTI_STREAM_ID)){
                rafti = t;
            } else if(rigidbody.rigid_body_name == std::to_string(GRASP_STREAM_ID)){
                grasp = t;
            }
        }

        tf2::Transform tfA, tfB;

        tf2::fromMsg(rafti.transform, tfA);
        tf2::fromMsg(grasp.transform, tfB);

        tf2::Transform tfRel = tfA.inverse() * tfB;

        geometry_msgs::msg::TransformStamped rel;

        current_subscribed_relative->transform = tf2::toMsg(rel);
        current_subscribed_relative->header.stamp = this->get_clock()->now();

        if(!initialised_tf){
            prev_relative_transform = *current_subscribed_relative;
            initialised_tf = true;
        }

        // relative_transform = tf_buffer_->lookupTransform(rafti, grasp, tf2::TimePointZero);
        // relative_transform.header.stamp = this->get_clock()->now();
        // // Convert to tf::Transform
        // tf2::fromMsg(relative_transform.transform, relative_tf);

    }

    // Process transforms.
    void process_transforms()
    {
        // Get the latest transforms.
        geometry_msgs::msg::TransformStamped relative_transform;
        tf2::Transform relative_tf;

        if(!current_subscribed_relative){
            RCLCPP_WARN(this->get_logger(), "No subscribed difference yet");
            return;
        }

        // try
        // {
        //     relative_transform = tf_buffer_->lookupTransform(std::to_string(RAFTI_STREAM_ID), std::to_string(GRASP_STREAM_ID), tf2::TimePointZero);
        //     relative_transform.header.stamp = this->get_clock()->now();
        //     // Convert to tf::Transform
        //     tf2::fromMsg(relative_transform.transform, relative_tf);
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        //     return;
        // }

        relative_transform = *current_subscribed_relative;

        // Find latest transform time and retrieve previous transform for twist calculation.
        rclcpp::Time latest_time = relative_transform.header.stamp;
        rclcpp::Time previous_time_target = latest_time - rclcpp::Duration(TWIST_COMPUTE_INTERVAL);

        // Get the previous transform.
        // geometry_msgs::msg::TransformStamped prev_relative_transform;
        // tf2::Transform prev_relative_tf;

        // try
        // {
        //     prev_relative_transform = tf_buffer_->lookupTransform(std::to_string(RAFTI_STREAM_ID), std::to_string(GRASP_STREAM_ID), previous_time_target);
        //     // Convert to tf::Transform
        //     tf2::fromMsg(prev_relative_transform.transform, prev_relative_tf);
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        //     return;
        // }

        // Get the actual previous time from the retrieved transform.
        rclcpp::Time previous_time = prev_relative_transform.header.stamp;
        double dt = (latest_time - previous_time).seconds();
        // RCLCPP_INFO(this->get_logger(), "dt: " << dt);
        // Check for zero or near-zero dt to avoid division by zero or large velocity spikes
        if (dt < MIN_DT) {
            RCLCPP_WARN(this->get_logger(), "Insufficiently large time difference between transforms for twist computation (dt = %g s). Skipping velocity calculation.", dt);
            return;
        }

        // Calculate twist (linear and angular velocity)
        // Linear velocity
        tf2::Vector3 relative_linear_vel = (relative_tf.getOrigin() - prev_relative_tf.getOrigin()) / dt;
        // Angular velocity
        tf2::Quaternion dq = relative_tf.getRotation() * prev_relative_tf.getRotation().inverse();
        dq.normalize();
        tf2::Vector3 relative_angular_vel = dq.getAxis() * (dq.getAngleShortestPath() / dt);

        // Convert transforms to pose messages.
        geometry_msgs::msg::PoseStamped relative_pose;
        relative_pose.header.stamp = latest_time;
        relative_pose.header.frame_id = std::to_string(RAFTI_STREAM_ID);
        relative_pose.pose.position.x = relative_transform.transform.translation.x;
        relative_pose.pose.position.y = relative_transform.transform.translation.y;
        relative_pose.pose.position.z = relative_transform.transform.translation.z;
        relative_pose.pose.orientation = relative_transform.transform.rotation;

        // Convert twist to twist message.
        geometry_msgs::msg::TwistStamped relative_twist;
        relative_twist.header.stamp = latest_time;
        relative_twist.header.frame_id = std::to_string(RAFTI_STREAM_ID);
        relative_twist.twist.linear.x = relative_linear_vel.x();
        relative_twist.twist.linear.y = relative_linear_vel.y();
        relative_twist.twist.linear.z = relative_linear_vel.z();
        relative_twist.twist.angular.x = relative_angular_vel.x();
        relative_twist.twist.angular.y = relative_angular_vel.y();
        relative_twist.twist.angular.z = relative_angular_vel.z();

        // Publish the relative pose.
        relative_pose_publisher_->publish(relative_pose);
        relative_twist_publisher_->publish(relative_twist);

        prev_relative_transform = relative_transform;
        // Convert to tf::Transform
        tf2::fromMsg(prev_relative_transform.transform, prev_relative_tf);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapTF2PostProcessor>());
  rclcpp::shutdown();
  return 0;
}