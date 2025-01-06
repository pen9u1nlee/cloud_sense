#ifndef __RGBDHANDLER_H_
#define __RGBDHANDLER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/RegistrationIcp.h>  // 引入 RegistrationICP 类型
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Features2d.h>  // 引入 Feature2D 类
#include <httplib.h>
#include <vector>
#include "sensor_data_serialization.h"
#include "image_subscriber/KeyFrameRGB.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <rtabmap_conversions/MsgConversion.h>

class RGBDHandler
{
public:
    RGBDHandler(ros::NodeHandle nh, int max_queue_size, int nb_local_keyframes, float keyframe_generation_ratio_threshold, int min_inliers);

    // 处理接收到的传感器数据
    void process_new_sensor_data();
    void compute_local_descriptors(std::shared_ptr<rtabmap::SensorData> &frame_data);
    bool generate_new_keyframe(std::shared_ptr<rtabmap::SensorData>& keyframe);
    void clear_sensor_data(std::shared_ptr<rtabmap::SensorData>& sensor_data);
    void send_keyframe(const std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::OdometryConstPtr> &keypoints_data);
    void sendSensorDataToCloud(const rtabmap::SensorData& sensor_data);
    void imageCallback(
        const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::ImageConstPtr& dph_msg,
        const sensor_msgs::CameraInfoConstPtr& camera_info,
        const nav_msgs::OdometryConstPtr& odom);

private:
    ros::NodeHandle nh_;

    
    // std::deque<std::shared_ptr<rtabmap::SensorData>> received_data_queue_;  // 存储 rtabmap::SensorData 对象
    std::deque<std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::OdometryConstPtr>> received_data_queue_;

    float keyframe_generation_ratio_threshold_; 
    int max_queue_size_ = 10;  // 队列最大大小
    int nb_local_keyframes_;  // 本地关键帧数量
    int min_inliers_;
    std::string base_frame_id_;

    
    // tf::TransformListener tf_listener_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 注册变换工具
    std::shared_ptr<rtabmap::RegistrationIcp> registration_;  // 使用智能指针管理
    std::shared_ptr<rtabmap::SensorData> previous_keyframe_;  // 记录上一个关键帧

    // 本地描述子映射
    std::map<double, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;
};

#endif // ifndef __RGBDHANDLER_H_
