#include <iostream>
#include <string>
#include <memory>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <httplib.h>
//#include <rtabmap/core/SensorData.h>
#include <boost/serialization/shared_ptr.hpp>
#include "sensor_data_serialization.h"

// 反序列化函数：接收客户端发送的传感器数据
void receiveSensorDataFromClient() {
    httplib::Server svr;

    // 定义接收传感器数据的 POST 请求
    svr.Post("/receive", [](const httplib::Request& req, httplib::Response& res) {
        // 反序列化请求体中的数据
        SerializableSensorData sensorData = deserializeSensorData(req.body);
        std::cout << "Received sensor data" << std::endl;
        sensorData.printInfo();
        // 处理接收到的数据（这里可以是进一步的处理，例如存储、分析等）

        // 向客户端返回响应
        res.set_content("Server received sensor data.", "text/plain");
    });

    // 启动服务器，监听在 8080 端口
    std::cout << "Server is running on http://localhost:8080/..." << std::endl;
    svr.listen("localhost", 8080);
}

int main() {
    // 启动服务器
    receiveSensorDataFromClient();
    return 0;
}
