#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class ForceVisualizer : public rclcpp::Node {
public:
    ForceVisualizer() : Node("force_visualizer") {

        // Declare parameters
        this->declare_parameter<std::string>("force_frame", "sensor_body");
        this->declare_parameter<std::string>("target_frame", "tibia_body");
        this->declare_parameter<bool>("debug_mode", false);

        // Create frame listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create force subscriber
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/ft_sensor/wrench_stamped", 10, std::bind(&ForceVisualizer::force_callback, this, std::placeholders::_1));

        // Create marker publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/force_marker", 10);
    }

private: 
    // Declare subscriber and publisher
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_; 
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Special case for transform listeners with the buffer
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // Debug variables
    int count; 

    // Force callback that is executed when a force message is received
    void force_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {

        // RCLCPP_INFO(this->get_logger(), "Received Message %i", count++);
        // Get frames
        std::string force_frame = "tibia_body";
        std::string target_frame = "optitrack";

        // Get latest transformation
        geometry_msgs::msg::TransformStamped transform_stamped; 
        try {
            transform_stamped = tf_buffer_->lookupTransform(target_frame, force_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s, %s", force_frame.c_str(), target_frame.c_str(), ex.what());
            return;
        }

        // Transform the force vector
        tf2::Vector3 force_vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z, 
            transform_stamped.transform.rotation.w);
        tf2::Vector3 transformed_force = tf2::quatRotate(q, force_vector);

        // Create visualization marker
        visualization_msgs::msg::Marker marker; 
        marker.header.frame_id = target_frame;
        marker.header.stamp = this->now();
        marker.ns = "force_visualization";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker start position
        marker.pose.position.x = transform_stamped.transform.translation.x;
        marker.pose.position.y = transform_stamped.transform.translation.y;
        marker.pose.position.z = transform_stamped.transform.translation.z;

        // Set marker orientation
        tf2::Quaternion arrow_orientation; 
        tf2::Vector3 default_arrow_orientation(1.0, 1.0, 1.0);
        arrow_orientation.setRotation(default_arrow_orientation.cross(transformed_force), default_arrow_orientation.angle(transformed_force));
        marker.pose.orientation.x = arrow_orientation.x(); 
        marker.pose.orientation.y = arrow_orientation.y(); 
        marker.pose.orientation.z = arrow_orientation.z(); 
        marker.pose.orientation.w = arrow_orientation.w(); 

        // Set arrow size
        double magnitude = transformed_force.length();
        marker.scale.x = magnitude * 0.1;  // Shaft length
        marker.scale.y = 0.02;             // Shaft width
        marker.scale.z = 0.02;             // Head width

        // set marker color based on magnitude
        marker.color.r = std::min(1.0, magnitude / 30.0);
        marker.color.g = 0.0;
        marker.color.b = 1.0 - std::min(1.0, magnitude / 30.0);
        marker.color.a = 1.0;

        // Publish marker
        marker_pub_->publish(marker);

    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceVisualizer>());
    rclcpp::shutdown();
    return 0;
}
