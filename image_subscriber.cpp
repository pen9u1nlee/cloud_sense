#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/RegistrationIcp.h>  // 引入 RegistrationICP 类型
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Features2d.h>  // 引入 Feature2D 类
#include <httplib.h>
#include <vector>
#include "sensor_data_serialization.h"

class RGBDHandler
{
public:
    RGBDHandler() : nh_("~"), max_queue_size_(10), nb_local_keyframes_(0), keyframe_generation_ratio_threshold_(0.99f), min_inliers_(10)
    {
        // 创建 ImageTransport 对象并订阅图像话题
        image_transport::ImageTransport it(nh_);
        rgb_sub_ = it.subscribe("/camera/rgb/image_raw", 1, &RGBDHandler::imageCallback, this);

        // 初始化 RegistrationICP（具体实现类）
        registration_ = std::make_shared<rtabmap::RegistrationIcp>();
    }

    // 图像回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
    try
    {
        //ROS_INFO("Received image with timestamp: %f and size: %dx%d", msg->header.stamp.toSec(), msg->width, msg->height);
        // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_image = cv_ptr->image;

        // 创建并设置 rtabmap::SensorData 对象
        rtabmap::SensorData sensor_data(rgb_image, msg->header.stamp.toSec());

        // 将传感器数据加入队列
        if (received_data_queue_.size() >= max_queue_size_)
        {
            // 在队列已满时删除最旧数据
            received_data_queue_.pop_front();  // 删除队列中的最旧数据
            ROS_INFO("Queue size exceeded. Removed oldest data. Current queue size: %ld", received_data_queue_.size());
        }

        // 将新的 sensor_data 加入队列
        received_data_queue_.push_back(std::make_shared<rtabmap::SensorData>(sensor_data));

        // 输出调试信息
        ROS_INFO("Added image to the queue. Queue size: %ld", received_data_queue_.size());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    }

    // 处理接收到的传感器数据
    void process_new_sensor_data()
    {
        if (!received_data_queue_.empty())
        {
            // 获取队列中的第一个传感器数据
            auto sensor_data = received_data_queue_.front();
            received_data_queue_.pop_front();
           
            ROS_INFO("Processing sensor data. Queue size after pop: %ld", received_data_queue_.size());

            // 处理有效数据
            if (sensor_data->isValid())
            {
                // 处理图像特征（计算局部描述子）
                compute_local_descriptors(sensor_data);

                // 判断是否生成关键帧
                bool generate_keyframe = generate_new_keyframe(sensor_data);
                if (generate_keyframe)
                {
                    nb_local_keyframes_++;
                    ROS_INFO("Keyframe generated. Local keyframes count: %d", nb_local_keyframes_);
                }

                // 清除处理过的传感器数据
                clear_sensor_data(sensor_data);

                // 如果生成了关键帧，则将其插入描述子映射
                if (generate_keyframe)
                {
                   local_descriptors_map_.insert({sensor_data->id(), sensor_data});
                   // 发送关键帧到云端
                   sendSensorDataToCloud(*sensor_data);  // 发送关键帧数据
                }
            }
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber rgb_sub_;  // 订阅 RGB 图像

    std::deque<std::shared_ptr<rtabmap::SensorData>> received_data_queue_;  // 存储 rtabmap::SensorData 对象
    float keyframe_generation_ratio_threshold_; 
    int max_queue_size_ = 10;  // 队列最大大小
    int nb_local_keyframes_;  // 本地关键帧数量
    int min_inliers_;
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
