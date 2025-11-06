#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

class PointCloudConverterNode : public rclcpp::Node
{
public:
    PointCloudConverterNode() : Node("pointcloud_converter_node")
    {
        // 声明参数
        this->declare_parameter("input_topic", "/points_raw");
        this->declare_parameter("output_topic", "/velodyne_points");
        this->declare_parameter("num_rings", 32);  // 激光雷达线数
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        num_rings_ = this->get_parameter("num_rings").as_int();
        
        // 创建订阅者和发布者
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudConverterNode::pointcloud_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Converter Node initialized");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Number of rings: %d", num_rings_);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
    {
        // 创建输出消息
        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        
        // 复制header
        output_msg->header = input_msg->header;
        output_msg->height = input_msg->height;
        output_msg->width = input_msg->width;
        output_msg->is_bigendian = input_msg->is_bigendian;
        output_msg->is_dense = input_msg->is_dense;
        
        // 设置新的字段：x, y, z, intensity, ring, time
        sensor_msgs::PointCloud2Modifier modifier(*output_msg);
        modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring", 1, sensor_msgs::msg::PointField::UINT16,
            "time", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        // 调整数据大小
        modifier.resize(input_msg->height * input_msg->width);
        
        // 创建迭代器读取输入数据
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*input_msg, "intensity");
        
        // 创建迭代器写入输出数据
        sensor_msgs::PointCloud2Iterator<float> iter_x_out(*output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y_out(*output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z_out(*output_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity_out(*output_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_out(*output_msg, "ring");
        sensor_msgs::PointCloud2Iterator<float> iter_time_out(*output_msg, "time");
        
        // 计算总点数
        size_t total_points = input_msg->height * input_msg->width;
        
        // 遍历所有点
        for (size_t i = 0; i < total_points; 
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity,
             ++iter_x_out, ++iter_y_out, ++iter_z_out, 
             ++iter_intensity_out, ++iter_ring_out, ++iter_time_out)
        {
            // 复制原始数据
            *iter_x_out = *iter_x;
            *iter_y_out = *iter_y;
            *iter_z_out = *iter_z;
            *iter_intensity_out = *iter_intensity;
            
            // 计算ring（基于height维度，假设每一行对应一个激光环）
            // 如果height是点数，width是线数，则ring = i % width
            // 如果width是点数，height是线数，则ring = i / width
            // 根据你的数据: height=1800, width=32，推测width是线数
            uint16_t ring = static_cast<uint16_t>(i % input_msg->width);
            *iter_ring_out = ring;
            
            // 计算time（假设每个点在扫描周期内均匀分布）
            // 对于旋转激光雷达，通常扫描周期为0.1秒（10Hz）
            float scan_period = 0.1f;  // 100ms
            float time = (static_cast<float>(i) / total_points) * scan_period;
            *iter_time_out = time;
        }
        
        // 发布转换后的点云
        publisher_->publish(*output_msg);
        
        // 记录日志（降低频率）
        static int count = 0;
        if (++count % 10 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Converted point cloud with %zu points", total_points);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    int num_rings_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverterNode>());
    rclcpp::shutdown();
    return 0;
}
