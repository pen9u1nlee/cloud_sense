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
#include "cloud_sense/KeyFrameRGB.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <rtabmap_conversions/MsgConversion.h>
#include "RGBDHandler.h"

void test_callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                   const sensor_msgs::ImageConstPtr& dph_msg,
                   const sensor_msgs::CameraInfoConstPtr& camera_info,
                   const nav_msgs::OdometryConstPtr& odom
                   );

RGBDHandler::RGBDHandler(ros::NodeHandle nh, int max_queue_size, int nb_local_keyframes, float keyframe_generation_ratio_threshold, int min_inliers) {
    
    nh_ = nh;
    max_queue_size_ = max_queue_size;
    nb_local_keyframes_ = nb_local_keyframes;
    keyframe_generation_ratio_threshold_ = keyframe_generation_ratio_threshold;
    min_inliers_ = min_inliers;

    image_transport::ImageTransport it(nh_);
    
    rgb_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/rgb/image_raw", 1);
    depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/depth/image_raw", 1);
    camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/camera/rgb/camera_info", 1);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/odom", 1);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *rgb_sub_, *depth_sub_, *camera_info_sub_, *odom_sub_);

    image_publisher_ = nh_.advertise<cloud_sense::KeyFrameRGB>("/keyframe_rgb", 20);

    // 初始化 RegistrationICP（具体实现类）
    registration_ = std::make_shared<rtabmap::RegistrationIcp>();
    // tf_listener_ = std::make_shared<tf::TransformListener>();

    std::cout << "rgbdhandler generated" << std::endl;
}


    

// 图像回调函数
void RGBDHandler::imageCallback(
    const sensor_msgs::ImageConstPtr& rgb_msg,
    const sensor_msgs::ImageConstPtr& dph_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info,
    const nav_msgs::OdometryConstPtr& odom) {
    
    try {
        //ROS_INFO("Received image with timestamp: %f and size: %dx%d", msg->header.stamp.toSec(), msg->width, msg->height);
        // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
        
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        // cv::Mat rgb_image = cv_ptr->image;

        // 创建并设置 rtabmap::SensorData 对象
        // rtabmap::SensorData sensor_data(rgb_image, msg->header.stamp.toSec());

        cv_bridge::CvImageConstPtr ptr_image = cv_bridge::toCvShare(rgb_msg);
        if (rgb_msg->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) != 0 &&
            rgb_msg->encoding.compare(sensor_msgs::image_encodings::MONO8) != 0)
        {
            if (rgb_msg->encoding.compare(sensor_msgs::image_encodings::MONO16) != 0)
            {
            ptr_image = cv_bridge::cvtColor(ptr_image, "bgr8");
            }
            else
            {
            ptr_image = cv_bridge::cvtColor(ptr_image, "mono8");
            }
        }
        ros::Time stamp = rtabmap_conversions::timestampFromROS(rgb_msg->header.stamp) > rtabmap_conversions::timestampFromROS(dph_msg->header.stamp) ? rgb_msg->header.stamp : dph_msg->header.stamp;

        rtabmap::Transform local_transform(0,0,0,0,0,0);


        cv_bridge::CvImageConstPtr ptr_depth = cv_bridge::toCvShare(dph_msg);

        rtabmap::CameraModel camera_model = rtabmap_conversions::cameraModelFromROS(*camera_info, local_transform);

        // copy data
        cv::Mat rgb, depth;
        ptr_image->image.copyTo(rgb);
        ptr_depth->image.copyTo(depth);

        auto data = std::make_shared<rtabmap::SensorData>(
            rgb, depth,
            camera_model,
            0,
            rtabmap_conversions::timestampFromROS(stamp));

        this->received_data_queue_.push_back(std::make_pair(data, odom));
        if (this->received_data_queue_.size() > max_queue_size_)
        {
            // Remove the oldest keyframes if we exceed the maximum size
            this->received_data_queue_.pop_front();
            ROS_WARN("RGBD: Maximum queue size (%d) exceeded, the oldest element was removed.", max_queue_size_);
        }
        // 输出调试信息
        ROS_INFO("Added image to the queue. Queue size: %ld", this->received_data_queue_.size());
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 处理接收到的传感器数据
void RGBDHandler::process_new_sensor_data(const ros::TimerEvent& e) {
    if (!received_data_queue_.empty()) {
        // 获取队列中的第一个传感器数据
        // std::pair<std::shared_ptr<rtabmap::SensorData>, std::shared_ptr<nav_msgs::OdometryConstPtr>> sensor_data = received_data_queue_.front();
        auto sensor_data = received_data_queue_.front();
        received_data_queue_.pop_front();
        
        ROS_INFO("Processing sensor data. Queue size after pop: %ld", received_data_queue_.size());
        if (sensor_data.first->isValid()) {
            compute_local_descriptors(sensor_data.first);

            bool generate_keyframe = generate_new_keyframe(sensor_data.first);
            if (generate_keyframe)
            {
                // Set keyframe ID
                sensor_data.first->setId(nb_local_keyframes_);
                nb_local_keyframes_++;

                // Send keyframe for loop detection
                send_keyframe(sensor_data);

            }

            clear_sensor_data(sensor_data.first);

            if (generate_keyframe)
            {
                local_descriptors_map_.insert({sensor_data.first->id(), sensor_data.first});
            }
        }   
    }
    else {
        ROS_DEBUG("the queue is empty");
    }
}


// 处理图像特征（计算局部描述子）
void RGBDHandler::compute_local_descriptors(std::shared_ptr<rtabmap::SensorData> &frame_data)
{
    // 解压图像数据
    frame_data->uncompressData();
    std::vector<cv::KeyPoint> kpts_from;

    // 获取图像数据
    cv::Mat image = frame_data->imageRaw();
    if (image.channels() > 1)
    {
        cv::Mat tmp;
        cv::cvtColor(image, tmp, cv::COLOR_BGR2GRAY);
        image = tmp;
    }

    cv::Mat depth_mask;
    if (!frame_data->depthRaw().empty())
    {
        // 检查深度图和图像尺寸是否匹配
        if (image.rows % frame_data->depthRaw().rows == 0 &&
            image.cols % frame_data->depthRaw().cols == 0 &&
            image.rows / frame_data->depthRaw().rows ==
                frame_data->imageRaw().cols / frame_data->depthRaw().cols)
        {
            depth_mask = rtabmap::util2d::interpolate(
                frame_data->depthRaw(),
                frame_data->imageRaw().rows / frame_data->depthRaw().rows, 0.1f);
        }
        else
        {
            ROS_WARN("RGB size (%dx%d) modulo depth size (%dx%d) is not 0. Ignoring depth mask for feature detection.",
                        frame_data->imageRaw().rows, frame_data->imageRaw().cols,
                        frame_data->depthRaw().rows, frame_data->depthRaw().cols);
        }
    }

    // 设置RTAB-Map参数
    rtabmap::ParametersMap registration_params;
    registration_params.insert(rtabmap::ParametersPair(
        rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));

    // 创建特征检测器（使用ORB或其他检测器）
    auto detector = rtabmap::Feature2D::create(registration_params);

    // 生成关键点
    std::vector<cv::KeyPoint> kpts = detector->generateKeypoints(image, depth_mask);
    // 生成描述子
    cv::Mat descriptors = detector->generateDescriptors(image, kpts);
    // 生成3D关键点
    std::vector<cv::Point3f> kpts3D = detector->generateKeypoints3D(*frame_data, kpts);

    // 设置特征数据到 frame_data
    frame_data->setFeatures(kpts, kpts3D, descriptors);
}

// 判断是否需要生成新关键帧
bool RGBDHandler::generate_new_keyframe(std::shared_ptr<rtabmap::SensorData>& keyframe)
{
    bool generate_new_keyframe = true;

    // 如果已经有关键帧
    if (nb_local_keyframes_ > 0)
    {
        try
        {
            // 使用之前的关键帧来计算当前帧和上一帧之间的变换
            rtabmap::RegistrationInfo reg_info;
            rtabmap::Transform t = registration_->computeTransformation(
                *keyframe, *previous_keyframe_, rtabmap::Transform(), &reg_info);

            // 如果变换有效，则进行内点比例判断
            if (!t.isNull())
            {
                if (float(reg_info.inliers) > keyframe_generation_ratio_threshold_ *
                    float(previous_keyframe_->keypoints().size()))
                {
                    // 如果内点数足够，避免生成新关键帧
                    generate_new_keyframe = false;
                }
            }
        }
        catch (std::exception &e)
        {
            ROS_WARN("Exception: Could not compute transformation for keyframe generation: %s", e.what());
        }
    }

    // 如果需要生成新关键帧，则更新 previous_keyframe_
    if (generate_new_keyframe)
    {
        previous_keyframe_ = keyframe;  // 直接将智能指针赋值
    }

    return generate_new_keyframe;
}

// 清除传感器数据中的昂贵部分
void RGBDHandler::clear_sensor_data(std::shared_ptr<rtabmap::SensorData>& sensor_data)
{
    // 清除传感器数据中的昂贵部分
    sensor_data->clearCompressedData();  // 清除压缩的数据
    sensor_data->clearRawData();  // 清除原始数据

    ROS_INFO("Cleared sensor data for timestamp: %f", sensor_data->stamp());
}


void RGBDHandler::send_keyframe(const std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::OdometryConstPtr> &keypoints_data) {
    cv::Mat rgb;
    keypoints_data.first->uncompressDataConst(&rgb, 0);

    // Image message
    cloud_sense::KeyFrameRGB kfmsg;
    sensor_msgs::Image img_msg;
    kfmsg.image_data = img_msg;
    
    img_msg.header.stamp = keypoints_data.second->header.stamp;
    cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
        img_msg.header, sensor_msgs::image_encodings::RGB8, rgb);
    
    
    image_bridge.toImageMsg(kfmsg.image_data);
    kfmsg.id = keypoints_data.first->id();

    image_publisher_.publish(kfmsg);
    std::printf("Keyframe ID: %d\n", kfmsg.id);
    std::vector<uint8_t> buffer;
    // 获取消息对象的字节大小，以便为缓冲区预留足够空间
    uint32_t buffer_size = ros::serialization::serializationLength(kfmsg);
    buffer.resize(buffer_size);
    // 使用ros::serialization::serialize进行序列化，将消息数据存入缓冲区
    ros::serialization::OStream stream(buffer.data(), buffer_size);
    ros::serialization::serialize(stream, kfmsg);
    sendDataToCloud(buffer); 
    std::cout << "Serialized data length: " << buffer.size() << std::endl;
}

ros::NodeHandle& RGBDHandler::getNH() {
    return nh_;
}

void RGBDHandler::sendDataToCloud(const std::vector<uint8_t>& serialized_data) {
    try {
        // 创建 HTTP 客户端对象，替换为你的云端地址
        httplib::Client cli("http://192.168.100.105:5000"); // 你的接收端地址

        // 发送 POST 请求，将数据作为 body 发送
        auto res = cli.Post("/receive", reinterpret_cast<const char*>(serialized_data.data()), serialized_data.size(), "application/octet-stream");

        // 检查响应
        if (res) {
            if (res->status == 200) {
                std::cout << "Server response: " << res->body << std::endl;
            } else {
                std::cerr << "Error: Received status code " << res->status << " from server. " 
                          << res->body << std::endl;
            }
        } else {
            std::cerr << "Error: No response from server." << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to send sensor data to cloud: " << e.what() << std::endl;
    }
}


// main函数，初始化ROS节点并启动数据处理
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber");
    RGBDHandler handler(ros::NodeHandle("~"), 30, 0, 0.85f, 10);

    handler.sync->registerCallback(boost::bind(&RGBDHandler::imageCallback, &handler, _1, _2, _3, _4));
    ros::Timer processorTimer = handler.getNH().createTimer(ros::Duration(0.02), 
                                                            boost::bind(&RGBDHandler::process_new_sensor_data, &handler, _1));
    std::cout << "timer created" << std::endl;

    // 保持节点运行
    ros::spin();

    return 0;
}

