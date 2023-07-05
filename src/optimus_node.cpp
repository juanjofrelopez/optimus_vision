#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <chrono>

#include "../include/optimus_vision/utils_lib.h"

ros::Publisher depth_publisher;

void depth_callback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depthImage = cv_ptr->image;
    cv::Mat depthValues;
    depthImage.convertTo(depthValues, CV_16U);
    //depthImage.convertTo(depthValues, CV_32S);
    
    // std::cout << depthValues.rows<< " " << depthValues.cols << std::endl;
    // std::cout<<"\x1b[1A"<<"\x1b[2K"<<"\r";
    
    // uint16_t imageHeight = msg->height; // rows
    // uint16_t imageWidth = msg->width; // cols
    
    auto start = std::chrono::high_resolution_clock::now();

    // Publish the depth image
    cv::Mat depthColormap = processDepth(depthValues);
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
    std::cout<<"\x1b[1A"<<"\x1b[2K"<<"\r";
    
    //std::cout<<depthImage.type()<<std::endl;
    //std::cout<<"\x1b[1A"<<"\x1b[2K"<<"\r";

    //sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthColormap).toImageMsg();
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",  depthColormap).toImageMsg();
    depth_publisher.publish(depth_msg);
}

void image_subscriber(){
    ros::NodeHandle nh;
    ros::Subscriber depth_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, depth_callback);
    depth_publisher = nh.advertise<sensor_msgs::Image>("optimus_depth", 10);
    ros::spin();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "optimus_vision");
    image_subscriber();
    ROS_INFO("Holaaaa");
    return 0;
}