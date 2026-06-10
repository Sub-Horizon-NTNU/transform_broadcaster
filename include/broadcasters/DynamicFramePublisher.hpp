
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


class DynamicFramePublisher{
    public:
    DynamicFramePublisher(rclcpp::Node::SharedPtr node,std::string parent, std::string child,
    const double x,const double y,const double z,const double roll, const double pitch, const double yaw)
    :   node_(node),
        parent_(parent),
        child_(child),
        x_translation_(x),y_translation_(y),z_translation_(z)
    {
        (void)roll; (void)pitch; (void)yaw;
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

        // PX4 publishes on /fmu/out/* with best-effort QoS (px4_ros_com pattern).
        rclcpp::QoS px4_qos(rclcpp::KeepLast(5));
        px4_qos.best_effort().durability_volatile();

        subscription_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", px4_qos,
            std::bind(&DynamicFramePublisher::handle_usv_odometry, this, std::placeholders::_1));
    }

    // VehicleOdometry.position is already NED and q is the body-FRD orientation w.r.t. the NED
    // reference frame (Hamilton, [w,x,y,z]). The TF tree here is world_ned -> usv_ned, so we
    // copy both straight through -- no ENU->NED reshuffle needed (unlike the old ArduPilot path).
    void handle_usv_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr odom){
        geometry_msgs::msg::TransformStamped usv_transform;
        usv_transform.header.stamp = node_->now();
        usv_transform.header.frame_id = parent_;
        usv_transform.child_frame_id = child_;

        usv_transform.transform.translation.x = odom->position[0] + x_translation_;
        usv_transform.transform.translation.y = odom->position[1] + y_translation_;
        usv_transform.transform.translation.z = odom->position[2] + z_translation_;

        // px4 q order is [w, x, y, z]; tf2::Quaternion takes (x, y, z, w).
        tf2::Quaternion orientation_ned(
            odom->q[1], odom->q[2], odom->q[3], odom->q[0]);
        orientation_ned.normalize();
        usv_transform.transform.rotation.x = orientation_ned.x();
        usv_transform.transform.rotation.y = orientation_ned.y();
        usv_transform.transform.rotation.z = orientation_ned.z();
        usv_transform.transform.rotation.w = orientation_ned.w();

        tf_broadcaster_->sendTransform(usv_transform);
    }


    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
        std::string parent_;
        std::string child_;
        double x_translation_,y_translation_,z_translation_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};
