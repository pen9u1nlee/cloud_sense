#ifndef SENSOR_DATA_SERIALIZATION_H
#define SENSOR_DATA_SERIALIZATION_H

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <rtabmap/core/SensorData.h>
#include <sstream>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <opencv2/opencv.hpp>

/*
  auto data = std::make_shared<rtabmap::SensorData>(
      rgb, depth,
      camera_model,
      0,
      rtabmap_conversions::timestampFromROS(stamp));

// RGB-D constructor
SensorData(
        const cv::Mat & rgb,
        const cv::Mat & depth,
        const CameraModel & cameraModel,
        int id = 0,
        double stamp = 0.0,
        const cv::Mat & userData = cv::Mat());

关键是序列化rgb depth stamp 三个变量。 在云上再构建sensordata类型就可以了
*/


namespace boost {
namespace serialization {

// 为 cv::Mat 实现序列化
template<class Archive>
void serialize(Archive & ar, cv::Mat & mat, const unsigned int version)
{
    int rows = mat.rows;
    int cols = mat.cols;
    int type = mat.type();
    ar & rows & cols & type;

    if (Archive::is_loading::value) {
        mat.create(rows, cols, type);  // 恢复矩阵大小和类型
    }

    // 序列化数据
    size_t data_size = mat.total() * mat.elemSize();
    std::vector<uchar> data(data_size);
    std::memcpy(data.data(), mat.data, data_size);
    ar & data;

    // 在加载时恢复矩阵数据
    if (Archive::is_loading::value) {
        std::memcpy(mat.data, data.data(), data_size);
    }
}
}  // namespace serialization
}  // namespace boost


class SerializableSensorData
{
public:
    // Constructor to initialize from SensorData
    SerializableSensorData(const cv::Mat & rgb,
        const cv::Mat & depth,
        int id = 0,
        double stamp = 0.0)
    {
        _rgb = rgb;
        _depth = depth;
        _id = id;
        _stamp = stamp;
        // Serialize other members of SensorData as needed
        // For example, images, laser scans, camera models, etc.
    }
    SerializableSensorData()
    {
        _id = 0;
        _stamp = 0.0;
    }
    // Getter函数
    cv::Mat getRgb() const { return _rgb; }
    cv::Mat getDepth() const { return _depth; }
    int getId() const { return _id; }
    double getStamp() const { return _stamp; }

        // 输出信息
    void printInfo() const {
        std::cout << "ID: " << _id << std::endl;
        std::cout << "Timestamp: " << _stamp << std::endl;
        std::cout << "RGB Image Size: " << _rgb.size() << std::endl;
        std::cout << "Depth Image Size: " << _depth.size() << std::endl;
        std::cout << "RGB Image: " << _rgb << std::endl;
    }

private:
    friend class boost::serialization::access;
    // Member variables to hold serialized data
    cv::Mat _rgb;
    cv::Mat _depth;
    int _id ;
    double _stamp ;
    // Other member variables for image, laser scan, etc.
    // You can choose which data to serialize based on your needs.
    // Boost serialization method
    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        // Serialize the basic data members
        ar & BOOST_SERIALIZATION_NVP(_rgb);
        ar & BOOST_SERIALIZATION_NVP(_depth);
        ar & BOOST_SERIALIZATION_NVP(_id);
        ar & BOOST_SERIALIZATION_NVP(_stamp);
        
        // Serialize other members (images, laser scans, etc.)
        // For example:
        // ar & BOOST_SERIALIZATION_NVP(_imageRaw);
        // ar & BOOST_SERIALIZATION_NVP(_laserScanRaw);
    }
};




/*class msg
{
    public:
        msg(){

        }

    void msg(std::shared_ptr<rtabmap::SensorData>& data):data_(data){

    }
    private:
        friend class boost::serialization::access;
        template <typename Archive>
        friend void serialize(Archive &ar,msg &m,const unsigned int version);
        std::shared_ptr<rtabmap::SensorData>& data_;

}

template <typename Archive>
void serialize(Archive &ar, msg &m, const unsigned int version)
{
    ar & m.data_;
}
*/

// 序列化函数 // 序列化函数：将传感器数据转换为字符串
inline std::string serializeSensorData(const SerializableSensorData& sensorData_delivery) {
    std::ostringstream oss;
    boost::archive::text_oarchive oa(oss);
    //rtabmap::SerializableSensorData serializableData(sensorData);
    oa << sensorData_delivery;
    return oss.str();
}

// 反序列化函数
inline SerializableSensorData deserializeSensorData(const std::string& str) {
    SerializableSensorData sensorData_delivery;    
    std::istringstream iss(str);
    boost::archive::text_iarchive ia(iss);
    ia >> sensorData_delivery;

    return sensorData_delivery;
}

#endif // SENSOR_DATA_SERIALIZATION_H
