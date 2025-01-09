#include <iostream>
#include <vector>
#include <httplib.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <cloud_sense/KeyFrameRGB.h>  // 包含 KeyFrameRGB 类型的头文件
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

void handle_receive(const httplib::Request& req, httplib::Response& res) {
    // 获取传入的二进制数据
    std::vector<uint8_t> received_data(req.body.begin(), req.body.end());

    // 创建 KeyFrameRGB 对象
    cloud_sense::KeyFrameRGB kfmsg;

    // 反序列化
    try {
        ros::serialization::IStream stream(received_data.data(), received_data.size());
        ros::serialization::deserialize(stream, kfmsg);

        std::printf("receive Keyframe ID: %d\n", kfmsg.id);

        // 打印收到的图像信息
        std::cout << "Image height: " << kfmsg.image_data.height << std::endl;
        std::cout << "Image width: " << kfmsg.image_data.width << std::endl;
        std::cout << "Image encoding: " << kfmsg.image_data.encoding << std::endl;
        std::cout << "Image data size: " << kfmsg.image_data.data.size() << std::endl;

        // 返回成功响应
        res.set_content("Received and processed KeyFrame.", "text/plain");
    } catch (const std::exception& e) {
        std::cerr << "Failed to deserialize data: " << e.what() << std::endl;
        res.status = 400;  // Bad Request
        res.set_content("Failed to deserialize the data.", "text/plain");
    }
}

int main() {
    // 创建 HTTP 服务器
    httplib::Server svr;

    // 处理 /receive 路径的 POST 请求
    svr.Post("/receive", handle_receive);

    // 启动服务器
    std::cout << "Server is running on http://192.168.100.105:5000" << std::endl;
    svr.listen("0.0.0.0", 5000);  // 监听 5000 端口
    return 0;
}
