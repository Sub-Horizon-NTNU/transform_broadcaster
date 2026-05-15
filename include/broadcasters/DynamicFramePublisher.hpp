
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class DynamicFramePublisher{
    //TODO: decouple class from the /ap topic
    public:
    DynamicFramePublisher(rclcpp::Node::SharedPtr node,std::string parent, std::string child,
    const double x,const double y,const double z,const double roll, const double pitch, const double yaw) 
    :   node_(node),
        parent_(parent),
        child_(child),
        x_translation_(x),y_translation_(y),z_translation_(z)
    {
         tf2::Matrix3x3 R_enu_ned(
            1.0, 0.0, 0,
            0.0, 1.0, 0,
            0, 0, 1.0
        );
        R_enu_ned.getRotation(transform_rotation);

        //transform_rotation.setRPY(M_PI,0.0,-M_PI/2.0);
        transform_rotation.normalize();

        tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        //USV
        subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ap/pose/filtered", rclcpp::SensorDataQoS(),
        std::bind(&DynamicFramePublisher::handle_usv_pose, this, std::placeholders::_1));
    }

    void handle_usv_pose(const geometry_msgs::msg::PoseStamped::SharedPtr pose){
        //ENU to NED
        tf2::Quaternion orientation;
        tf2::fromMsg(pose->pose.orientation,orientation);
        double roll,pitch,yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        tf2::Quaternion orientation_ned;
        orientation_ned.setRPY(pitch,roll,-yaw+M_PI/2.0);
        //orientation =  orientation*transform_rotation;
        orientation_ned.normalize();
    
        geometry_msgs::msg::TransformStamped usv_transform;
        usv_transform.header.stamp = node_->now();
        usv_transform.header.frame_id = parent_;
        usv_transform.child_frame_id = child_;
        usv_transform.transform.rotation.x = orientation_ned.x();
        usv_transform.transform.rotation.y = orientation_ned.y();
        usv_transform.transform.rotation.z = orientation_ned.z();
        usv_transform.transform.rotation.w = orientation_ned.w();
        usv_transform.transform.translation.x =  pose->pose.position.y; + x_translation_;
        usv_transform.transform.translation.y =  pose->pose.position.x; + y_translation_;
        usv_transform.transform.translation.z = -pose->pose.position.z;  + z_translation_;
        tf_broadcaster_->sendTransform(usv_transform);
    }

   
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
        std::string parent_;
        std::string child_;
        double x_translation_,y_translation_,z_translation_;
        tf2::Quaternion transform_rotation;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};