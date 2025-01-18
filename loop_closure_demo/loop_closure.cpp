// #include <ros/ros.h>
// #include <rtabmap/core/RegistrationIcp.h>
// #include <rtabmap/core/RegistrationVis.h>
// #include <rtabmap/core/Transform.h>
// #include <rtabmap/core/SensorData.h>
// #include <rtabmap/core/Features2d.h>
// #include <rtabmap/core/Parameters.h>
// #include <rtabmap_conversions/MsgConversion.h>
// #include <std_msgs/String.h>
// #include <sensor_msgs/Image.h>
// #include <opencv2/opencv.hpp>
// #include <deque>
// #include <memory>
// #include "cloud_sense/KeyFrameRGB.h"

// class LoopClosureHandler {
// public:
//     LoopClosureHandler(ros::NodeHandle& nh,int min_inliers_)
//     : nh_(nh) {
//         // 发布匹配结果到PG0的主题
//         loop_closure_publisher_ = nh_.advertise<std_msgs::String>("/loop_closure_result", 10);
//         min_inliers_ =  min_inliers_;

//         // 初始化ICP匹配器
//         // registration_ = std::make_shared<rtabmap::RegistrationIcp>();

//         rtabmap::ParametersMap registration_params;
//         registration_params.insert(rtabmap::ParametersPair(
//         rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)));
//         registration_params.insert(rtabmap::ParametersPair(
//         rtabmap::Parameters::kVisMaxFeatures(), "500"));  // 增加最大特征数
//         registration_.parseParameters(registration_params);
//     }

//     // keyframeCallback 改为类的成员函数
//     void keyframeCallback(const cloud_sense::KeyFrameRGB::ConstPtr& msg) {
//         try {
//             // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式
//             cv_bridge::CvImageConstPtr rgb_image_ptr = cv_bridge::toCvShare(boost::make_shared<sensor_msgs::Image>(msg->image_data), sensor_msgs::image_encodings::BGR8);
//             cv::Mat rgb_image = rgb_image_ptr->image;  // 得到 OpenCV 图像

//             // 创建一个新的 rtabmap::SensorData 对象
//             std::shared_ptr<rtabmap::SensorData> new_keyframe = std::make_shared<rtabmap::SensorData>();

//             // 设置 rtabmap::SensorData 对象的图像数据
//             new_keyframe->setImageRaw(rgb_image);

            
//             // 将新关键帧加入队列
//             keyframe_queue_.push_back(new_keyframe);

//             ROS_INFO("Received new keyframe with ID: %d, queue size: %ld", msg->id, keyframe_queue_.size());
//         } catch (const cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

//     // 处理关键帧回环检测
//     void process_loop_closure() {
//         if (keyframe_queue_.size() >= 2) {
//             std::shared_ptr<rtabmap::SensorData> current_keyframe = keyframe_queue_.back();
//             keyframe_queue_.pop_back();
//             std::shared_ptr<rtabmap::SensorData> previous_keyframe = keyframe_queue_.back();
//             keyframe_queue_.pop_back();

//             try {
//                 // 执行回环检测，获取当前帧和上一帧之间的变换
//                 rtabmap::RegistrationInfo reg_info;
//                 rtabmap::Transform transform = registration_.computeTransformation(
//                     *current_keyframe, *previous_keyframe, rtabmap::Transform(), &reg_info);
//                 ROS_INFO("Transformation computed: %s", transform.isNull() ? "null" : "valid");
                

//                 // 如果变换有效且内点数超过一定阈值，判定为回环
//                 if (!transform.isNull() && reg_info.inliers > 0) {
//                     ROS_INFO("Loop Closure Detected between frame %d and frame %d with %d inliers.",
//                              previous_keyframe->id(), current_keyframe->id(), reg_info.inliers);

//                     // 将回环信息发布到PG0
//                     publish_loop_closure_result(current_keyframe->id(), previous_keyframe->id(), transform);
//                 }
//             } catch (const std::exception& e) {
//                 ROS_WARN("Loop closure detection failed: %s", e.what());
//             }
//         }
//     }

//     // 发布回环检测结果
//     void publish_loop_closure_result(int current_frame_id, int previous_frame_id, const rtabmap::Transform& transform) {
//         std_msgs::String loop_closure_msg;
//         std::stringstream ss;

//         // 这里假设你要发送的格式为 JSON 或者简单的字符串
//         // 获取旋转矩阵并计算欧拉角
//         cv::Mat rotation_matrix = transform.rotationMatrix();
//         cv::Vec3d euler_angles;
//         cv::Rodrigues(rotation_matrix, euler_angles);  // 使用 Rodrigues 计算欧拉角
//         double roll = euler_angles[0];
//         double pitch = euler_angles[1];
//         double yaw = euler_angles[2];
        
//         ss << "Loop closure detected between frame " << current_frame_id << " and frame " << previous_frame_id 
//            << " with transform [x=" << transform.x() << ", y=" << transform.y() << ", z=" << transform.z() 
//            << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << "]";

//         loop_closure_msg.data = ss.str();
//         loop_closure_publisher_.publish(loop_closure_msg);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Publisher loop_closure_publisher_;
//     rtabmap::RegistrationVis registration_;
//     std::deque<std::shared_ptr<rtabmap::SensorData>> keyframe_queue_;  // keyframe_queue 成为成员
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "loop_closure");
//     ros::NodeHandle nh;

//     // 初始化回环检测处理类
//     LoopClosureHandler loop_closure_handler(nh,10);

//     // 创建ROS订阅者，订阅关键帧数据
//     ros::Subscriber keyframe_sub = nh.subscribe("/keyframe_rgb", 10, &LoopClosureHandler::keyframeCallback, &loop_closure_handler);

//     // 订阅回环检测处理回调函数
//     while (ros::ok()) {
//         // 执行回环检测
//         loop_closure_handler.process_loop_closure();

//         // 保证回调函数能够被执行
//         ros::spinOnce();
//     }

//     return 0;
// }


#include <ros/ros.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include "cloud_sense/KeyFrameRGB.h"

class LoopClosureHandler {
public:
    LoopClosureHandler(ros::NodeHandle& nh, int min_inliers_)
        : nh_(nh) {
        // 发布匹配结果到PG0的主题
        loop_closure_publisher_ = nh_.advertise<std_msgs::String>("/loop_closure_result", 10);
        
        // 初始化 Loop Closure 检测器
        loop_closure_detector_.init(rtabmap::ParametersMap(), "");
        loop_closure_detector_.parseParameters({
            {rtabmap::Parameters::kVisMinInliers(), std::to_string(min_inliers_)},
            {rtabmap::Parameters::kVisMaxFeatures(), "500"}  // 增加最大特征数
        });
    }

    // keyframeCallback 改为类的成员函数
    void keyframeCallback(const cloud_sense::KeyFrameRGB::ConstPtr& msg) {
        try {
            // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式
            cv_bridge::CvImageConstPtr rgb_image_ptr = cv_bridge::toCvShare(boost::make_shared<sensor_msgs::Image>(msg->image_data), sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb_image = rgb_image_ptr->image;  // 得到 OpenCV 图像

            // 创建一个新的 rtabmap::SensorData 对象
            std::shared_ptr<rtabmap::SensorData> new_keyframe = std::make_shared<rtabmap::SensorData>();

            // 设置 rtabmap::SensorData 对象的图像数据
            new_keyframe->setImageRaw(rgb_image);

            // 将新关键帧加入队列
            keyframe_queue_.push_back(new_keyframe);

            // // 处理关键帧并存储到数据库
            // if (keyframe_queue_.size() < 2){
            //     loop_closure_detector_.addSignature(*new_keyframe);}


            cv::imshow("new_keyframe", rgb_image);
            cv::waitKey(1);
            

            ROS_INFO("Received new keyframe with ID: %d, queue size: %ld", msg->id, keyframe_queue_.size());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // 处理关键帧回环检测
    void process_loop_closure() {
        if (keyframe_queue_.size() >= 2) {
            std::shared_ptr<rtabmap::SensorData> current_keyframe = keyframe_queue_.back();

            try {
                // 使用loopClosureDetector进行回环检测
                if (loop_closure_detector_.process(current_keyframe->imageRaw(), current_keyframe->id())) {
                    int loop_closure_id = loop_closure_detector_.getLoopClosureId();
                    ROS_INFO("Loop closure ID: %d", loop_closure_id);  // 输出回环检测ID，调试用

                    // 如果找到了回环
                    if (loop_closure_id > 0) {
                        ROS_INFO("Loop Closure Detected between frame %d and frame %d",
                            current_keyframe->id(), loop_closure_id);

                    // 发布回环信息
                        publish_loop_closure_result(current_keyframe->id(), loop_closure_id);
                    }

                }
                    // else{
                    //     ROS_INFO("NO Loop Closure Detected");
                    // }
            } catch (const std::exception& e) {
            ROS_WARN("Loop closure detection failed: %s", e.what());
            }
            
        }
}



    // 发布回环检测结果
    void publish_loop_closure_result(int current_frame_id, int previous_frame_id) {
        std_msgs::String loop_closure_msg;
        std::stringstream ss;
        ss << "Loop closure detected between frame " << current_frame_id << " and frame " << previous_frame_id;
        loop_closure_msg.data = ss.str();
        loop_closure_publisher_.publish(loop_closure_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher loop_closure_publisher_;
    rtabmap::Rtabmap loop_closure_detector_;  // 使用 rtabmap::Rtabmap 进行回环检测
    std::deque<std::shared_ptr<rtabmap::SensorData>> keyframe_queue_;  // keyframe_queue 成为成员
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "loop_closure");
    ros::NodeHandle nh;

    // 初始化回环检测处理类
    LoopClosureHandler loop_closure_handler(nh, 1);

    // 创建ROS订阅者，订阅关键帧数据
    ros::Subscriber keyframe_sub = nh.subscribe("/keyframe_rgb", 10, &LoopClosureHandler::keyframeCallback, &loop_closure_handler);

    // 订阅回环检测处理回调函数
    while (ros::ok()) {
        // 执行回环检测
        loop_closure_handler.process_loop_closure();

        // 保证回调函数能够被执行
        ros::spinOnce();
    }

    return 0;
}





