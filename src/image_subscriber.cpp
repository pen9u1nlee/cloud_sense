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
    RGBDHandler() : nh_("~"), max_queue_size_(10), nb_local_keyframes_(0), keyframe_generation_ratio_threshold_(0.9f), min_inliers_(10)
    {
        // 创建 ImageTransport 对象并订阅图像话题
        // image_transport::Subscriber rgb_sub_;  // 订阅 RGB 图像
        // image_transport::Subscriber depth_sub_; // 订阅 D 图像

        image_transport::ImageTransport it(nh_);
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub_(nh_, "/camera/rgb/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh_, "/camera/depth/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_(nh_, "/camera/camera_info", 1);
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_(nh_, "/odom", 1);

        typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, 
        sensor_msgs::Image, 
        sensor_msgs::CameraInfo, 
        nav_msgs::Odometry> SyncPolicy;

        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub_, depth_sub_, camera_info_sub_, odom_sub_);

        sync.registerCallback(boost::bind(&RGBDHandler::imageCallback, _1, _2, _3, _4));

        // rgb_sub_ = it.subscribe("/camera/rgb/image_raw", 1, &RGBDHandler::imageCallback, this);

        // 初始化 RegistrationICP（具体实现类）
        registration_ = std::make_shared<rtabmap::RegistrationIcp>();
        tf_listener_ = std::make_shared<tf::TransformListener>();
    }

    // 图像回调函数
    // image_rect_rgb,
    // const sensor_msgs::msg::Image::ConstSharedPtr image_rect_depth,
    // const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_rgb,
    // const nav_msgs::msg::Odometry::ConstSharedPtr odom
    void imageCallback(
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
            if (base_frame_id_ != "") {
                // here error occurs
                local_transform = rtabmap_conversions::getTransform(base_frame_id_, rgb_msg->header.frame_id, stamp, *tf_listener_, 0.1);
                if (local_transform.isNull()) {
                    return;
                }
            }

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

            received_data_queue_.push_back(std::make_pair(data, odom));
            if (received_data_queue_.size() > max_queue_size_)
            {
                // Remove the oldest keyframes if we exceed the maximum size
                received_data_queue_.pop_front();
                ROS_WARN("RGBD: Maximum queue size (%d) exceeded, the oldest element was removed.", max_queue_size_);
            }
            // 输出调试信息
            ROS_INFO("Added image to the queue. Queue size: %ld", received_data_queue_.size());
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }


        
                // // 处理图像特征（计算局部描述子）
                // compute_local_descriptors(sensor_data);

                // // 判断是否生成关键帧
                // bool generate_keyframe = generate_new_keyframe(sensor_data);
                // if (generate_keyframe)
                // {
                //     nb_local_keyframes_++;
                //     ROS_INFO("Keyframe generated. Local keyframes count: %d", nb_local_keyframes_);
                // }

                // // 如果生成了关键帧，则将其插入描述子映射
                // if (generate_keyframe)
                // {
                //    local_descriptors_map_.insert({sensor_data->id(), sensor_data});
                //    // 发送关键帧到云端
                //    send_keyframe(sensor_data);
                //    // sendSensorDataToCloud(*sensor_data);  // 发送关键帧数据
                // }

                // // 清除处理过的传感器数据
                // clear_sensor_data(sensor_data);
            

    // 处理接收到的传感器数据
    void process_new_sensor_data() {
        if (!received_data_queue_.empty())
        {
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
    }


private:
    ros::NodeHandle nh_;

    
    // std::deque<std::shared_ptr<rtabmap::SensorData>> received_data_queue_;  // 存储 rtabmap::SensorData 对象
    std::deque<std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::OdometryConstPtr>> received_data_queue_;

    float keyframe_generation_ratio_threshold_; 
    int max_queue_size_ = 10;  // 队列最大大小
    int nb_local_keyframes_;  // 本地关键帧数量
    int min_inliers_;
    std::string base_frame_id_;

    
    tf::TransformListener tf_listener_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 注册变换工具
    std::shared_ptr<rtabmap::RegistrationIcp> registration_;  // 使用智能指针管理
    std::shared_ptr<rtabmap::SensorData> previous_keyframe_;  // 记录上一个关键帧

    // 处理图像特征（计算局部描述子）
    void compute_local_descriptors(std::shared_ptr<rtabmap::SensorData> &frame_data)
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
    bool generate_new_keyframe(std::shared_ptr<rtabmap::SensorData>& keyframe)
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
    void clear_sensor_data(std::shared_ptr<rtabmap::SensorData>& sensor_data)
    {
        // 清除传感器数据中的昂贵部分
        //sensor_data->clearCompressedData();  // 清除压缩的数据
        //sensor_data->clearRawData();  // 清除原始数据

        ROS_INFO("Cleared sensor data for timestamp: %f", sensor_data->stamp());
    }

    // 本地描述子映射
    std::map<double, std::shared_ptr<rtabmap::SensorData>> local_descriptors_map_;

    void send_keyframe(const std::pair<std::shared_ptr<rtabmap::SensorData>, nav_msgs::OdometryConstPtr> &keypoints_data) {
        cv::Mat rgb;
        keypoints_data.first->uncompressDataConst(&rgb, 0);

        // Image message
        image_subscriber::KeyFrameRGB kfmsg;
        sensor_msgs::Image img_msg;
        kfmsg.image_data = img_msg;
        
        img_msg.header.stamp = keypoints_data.second->header.stamp;
        cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
            img_msg.header, sensor_msgs::image_encodings::RGB8, rgb);
        
        
        image_bridge.toImageMsg(kfmsg.image_data);
        kfmsg.id = keypoints_data.first->id();

        std::vector<uint8_t> buffer;
        // 获取消息对象的字节大小，以便为缓冲区预留足够空间
        std::cout << "Serialized data length: " << buffer.size() << std::endl;
        uint32_t buffer_size = ros::serialization::serializationLength(kfmsg);
        buffer.resize(buffer_size);
        // 使用ros::serialization::serialize进行序列化，将消息数据存入缓冲区
        ros::serialization::OStream stream(buffer.data(), buffer_size);
        ros::serialization::serialize(stream, kfmsg);

        std::cout << "Serialized data length: " << buffer.size() << std::endl;
        // delete *buffer;
    }

    void sendSensorDataToCloud(const rtabmap::SensorData& sensor_data)
    {
      try
      {
        // 从 SensorData 中提取 RGB 图像和时间戳
        cv::Mat rgb = sensor_data.imageRaw();  // 获取 RGB 图像
        double timestamp = sensor_data.stamp();  // 获取时间戳

        // 如果需要深度图像，可以通过 sensor_data.depthRaw() 获取
        cv::Mat depth = sensor_data.depthRaw();  // 获取深度图像（可选）

        // 打印出图像和时间戳以确认
        std::cout << "Timestamp: " << timestamp << std::endl;
        std::cout << "RGB Image Size: " << rgb.size() << std::endl;
        //std::cout << "Depth Image Size: " << depth.size() << std::endl;

        // 创建 SerializableSensorData 对象（可以用来序列化）
        SerializableSensorData sensorData(rgb, depth, sensor_data.id(), timestamp);

        // 序列化数据
        std::string serializedData = serializeSensorData(sensorData);

        // 创建 HTTP 客户端对象
        httplib::Client cli("http://localhost:8080");

        // 发送 POST 请求，将数据作为 body 发送
        auto res = cli.Post("/receive", serializedData, "application/octet-stream");

        // 检查响应
         if (res && res->status == 200) {
            std::cout << "Server response: " << res->body << std::endl;
         } else {
            std::cerr << "Error: " << (res ? std::to_string(res->status) : "No response") << std::endl;
         }
      }
      catch (const std::exception& e)
      {
        std::cerr << "Failed to send sensor data to cloud: " << e.what() << std::endl;
      }
    }

};
// main函数，初始化ROS节点并启动数据处理
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subscriber");
    RGBDHandler handler;

    // 保持节点运行
    ros::Rate loop_rate(10);  // 设定循环频率
    while (ros::ok())
    {
        // 处理接收到的传感器数据
        handler.process_new_sensor_data();

        // 处理ROS回调
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
