#include <iostream>
#include <string>
#include <memory>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <httplib.h>
//#include <rtabmap/core/SensorData.h>
#include <boost/serialization/shared_ptr.hpp>
#include "sensor_data_serialization.h"


#include <random>

void sendSensorDataToCloud() {
        // 使用随机数生成器生成随机数据
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 255);  // 用于生成RGB值

    // 随机生成一个 480x640 的RGB图像
    cv::Mat rgb(480, 640, CV_8UC3);  // 480行，640列，3通道（RGB）
    for (int i = 0; i < rgb.rows; ++i) {
        for (int j = 0; j < rgb.cols; ++j) {
            //rgb.at<cv::Vec3b>(i, j) = cv::Vec3b(dist(gen), dist(gen), dist(gen));  // 随机RGB值
            rgb.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 1, 2);
        }
    }

    // 随机生成一个 480x640 的深度图像
    cv::Mat depth(480, 640, CV_16U);  // 16位无符号深度图像
    for (int i = 0; i < depth.rows; ++i) {
        for (int j = 0; j < depth.cols; ++j) {
            depth.at<unsigned short>(i, j) = dist(gen);  // 随机深度值
        }
    }

    // 随机生成ID和时间戳
    std::uniform_int_distribution<> id_dist(1, 100);
    std::uniform_real_distribution<> timestamp_dist(0.0, 1000.0);

    int id = id_dist(gen);  // 随机ID
    double timestamp = timestamp_dist(gen);  // 随机时间戳

    // 创建SerializableSensorData对象
    SerializableSensorData sensorData(rgb, depth, id, timestamp);



    // 假设我们有一个传感器数据对象
    //std::shared_ptr<rtabmap::SensorData> sensorData = std::make_shared<rtabmap::SensorData>();
    //SerializableSensorData m(sensorData);
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

int main() {
    // 等待云端服务器启动
    std::cout << "Waiting for cloud server to start..." << std::endl;

    // 等待服务器启动，稍作延迟
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 发送数据到云端
    sendSensorDataToCloud();

    return 0;
}
