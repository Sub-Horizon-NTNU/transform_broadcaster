#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher
{
public:
StaticFramePublisher(rclcpp::Node::SharedPtr node,const std::string parent,const std::string child, const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw) : 
  node_(node),
  parent_(parent),
  child_(child),
  x_translation_(x),
  y_translation_(y),
  z_translation_(z)
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    
    //q.setRPY(M_PI,0.0,M_PI/2.0);
    q_.setRPY(roll,pitch,yaw);
    // Publish static transforms once at startup
    this->make_transforms();
  }

private:
  void make_transforms(){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = node_->get_clock()->now();
    t.header.frame_id = parent_;
    t.child_frame_id = child_;

    t.transform.translation.x = x_translation_;
    t.transform.translation.y = y_translation_;
    t.transform.translation.z = z_translation_;
    t.transform.rotation.x = q_.x();
    t.transform.rotation.y = q_.y();
    t.transform.rotation.z = q_.z();
    t.transform.rotation.w = q_.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Node::SharedPtr node_;
  tf2::Quaternion q_;
  std::string parent_;
  std::string child_;
  double x_translation_,y_translation_,z_translation_;
};
