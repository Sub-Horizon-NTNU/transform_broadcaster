#pragma once

#include "rclcpp/rclcpp.hpp"
#include "broadcasters/StaticFramePublisher.hpp"
#include "broadcasters/DynamicFramePublisher.hpp"
#include <memory>

class TransformBroadcasterNode: public rclcpp::Node{
    public:
    TransformBroadcasterNode(): Node("transform_broadcaster") {}

    void init(){
        this->declare_parameter("camera_offset_x",0.2);
        this->declare_parameter("camera_offset_y",0.0);
        this->declare_parameter("camera_offset_z",-0.1);

        double cam_offset_x_ = this->get_parameter("camera_offset_x").as_double();
        double cam_offset_y_ = this->get_parameter("camera_offset_y").as_double();
        double cam_offset_z_ = this->get_parameter("camera_offset_z").as_double();
        

        static_frame_publisher_1 = std::make_unique<StaticFramePublisher>(
            this->shared_from_this(),
            "map","world_ned",0.0,0.0,0.0,M_PI,0.0,0.0
        );

        dynamic_frame_publisher_ = std::make_unique<DynamicFramePublisher>(
            this->shared_from_this(),"world_ned","usv_ned",0.0,0.0,0.0,0.0, 0.0, -M_PI/2
        );

        //Transformation between usv (NED) and camera (Z_UP X_FWD)
        static_frame_publisher_2 = std::make_unique<StaticFramePublisher>(
            this->shared_from_this(),
            "usv_ned","camera",cam_offset_x_,cam_offset_y_,cam_offset_z_,M_PI,0.0,0.0
        );
        RCLCPP_INFO(this->get_logger(),"Tranform broadcaster started");
    }

    private:
    std::unique_ptr<StaticFramePublisher> static_frame_publisher_1; 
    std::unique_ptr<StaticFramePublisher> static_frame_publisher_2;

    std::unique_ptr<DynamicFramePublisher> dynamic_frame_publisher_; 
};
