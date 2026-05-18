#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "vector"
#include "cstring"
#include "algorithm"
#include "cmath"
#include "cstddef"
#include "cstdint"
#include "limits"
#include "string"

using namespace std::chrono_literals;

// 自定义节点类
class CloudAccumulator : public rclcpp::Node
{
public:
    CloudAccumulator() : Node("cloud_accumulator")
    {   
        RCLCPP_INFO(this->get_logger(), "Cloud accumulator node started.");
        // 配置qos，确保数据能够传输，需要确认slam-toolbox用的是什么qos
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.reliable();

        // 创建订阅方， 订阅机器狗的 /utlidar/cloud_deskewed 话题
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_deskewed",
            rclcpp::SensorDataQoS(),
            std::bind(&CloudAccumulator::cloud_callback, this, std::placeholders::_1)
        );

        // 创建发布方，发布机器狗的 /utlidar/cloud_accumulated 话题
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_accumulated",
            qos
        );

        // 创建定时器，定时发布数据
        timer_ = this->create_wall_timer(
            25ms,
            std::bind(&CloudAccumulator::timer_callback, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_; // 创建存储点云的容器(注意点云数据是只读的，不能被修改)
    sensor_msgs::msg::PointCloud2 accumulated_cloud_; // 创建累积点云数据

    const size_t max_clouds_ = 25;  // 最多存储25帧点云数据
    const float min_height_ = 0.2f; // 最小高度阈值
    const float max_height_ = 0.8f; // 最大高度阈值

    static bool fields_are_compatible(
        const sensor_msgs::msg::PointCloud2 &lhs,
        const sensor_msgs::msg::PointCloud2 &rhs)
    {
        if (lhs.point_step != rhs.point_step || lhs.fields.size() != rhs.fields.size())
        {
            return false;
        }
        for (size_t i = 0; i < lhs.fields.size(); ++i)
        {
            const auto &left = lhs.fields[i];
            const auto &right = rhs.fields[i];
            if (
                left.name != right.name ||
                left.offset != right.offset ||
                left.datatype != right.datatype ||
                left.count != right.count)
            {
                return false;
            }
        }
        return true;
    }

    static bool find_float32_field_offset(
        const sensor_msgs::msg::PointCloud2 &cloud,
        const std::string &name,
        size_t &offset)
    {
        for (const auto &field : cloud.fields)
        {
            if (
                field.name == name &&
                field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                field.count >= 1)
            {
                offset = static_cast<size_t>(field.offset);
                return true;
            }
        }
        return false;
    }

    size_t valid_point_count(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        if (cloud.point_step == 0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Skipping PointCloud2 with point_step=0.");
            return 0;
        }

        const size_t points_by_data = cloud.data.size() / cloud.point_step;
        const uint64_t declared_points =
            static_cast<uint64_t>(cloud.width) * static_cast<uint64_t>(cloud.height);
        if (declared_points == 0)
        {
            return points_by_data;
        }

        const size_t declared = static_cast<size_t>(
            std::min<uint64_t>(declared_points, static_cast<uint64_t>(std::numeric_limits<size_t>::max())));
        if (declared > points_by_data)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "PointCloud2 data is shorter than declared dimensions; declared=%zu data_points=%zu.",
                declared, points_by_data);
        }
        return std::min(declared, points_by_data);
    }

    // 回调函数（注意参数使用的是ConstSharedPtr类型, 防止点云信息被修改）
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        clouds_.push_back(cloud_msg);

        // 如果点云的数量超过max_clouds_，则删除最老的点云数据
        if(clouds_.size() > max_clouds_)
        {
            clouds_.erase(clouds_.begin());
        }

        // 如果没有点云数据，则返回
        if(clouds_.empty())
        {
            return;
        }

        // 合并所有点云
        auto merged_cloud = merged_clouds();

        // 高度过滤
        auto filtered_cloud = filter_cloud(*merged_cloud);

        // 更新积累的点云
        accumulated_cloud_ = filtered_cloud;
        accumulated_cloud_.header.frame_id = "odom";    // 设置坐标系
    }

    // 合并多个点云的函数
    std::shared_ptr<sensor_msgs::msg::PointCloud2> merged_clouds()
    {   
        // std::shared_ptr<sensor_msgs::msg::PointCloud2>
        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

        *merged_cloud = *clouds_[0]; // 用第一个点云初始化 “合并后点云”  // *merged_cloud = *clouds_.front();
        merged_cloud->height = 1;
        merged_cloud->width = 0;
        merged_cloud->row_step = 0;
        merged_cloud->data.clear();

        // 遍历点云，逐个合并到基础点云中；遇到不同字段布局时跳过，避免混合 PointCloud2 格式导致越界。
        for (size_t i = 0; i < clouds_.size(); i++)
        {
            if (!fields_are_compatible(*clouds_[0], *clouds_[i]))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Skipping incompatible PointCloud2 frame while accumulating clouds.");
                continue;
            }

            const size_t point_count = valid_point_count(*clouds_[i]);
            const size_t byte_count = point_count * clouds_[i]->point_step;
            if (byte_count == 0)
            {
                continue;
            }
            if (
                point_count >
                static_cast<size_t>(std::numeric_limits<uint32_t>::max() - merged_cloud->width))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Accumulated PointCloud2 width overflow; dropping remaining frames.");
                break;
            }

            // 拼接当前点云的有效数据。
            merged_cloud->data.insert(merged_cloud->data.end(),
                    clouds_[i]->data.begin(), 
                    clouds_[i]->data.begin() + static_cast<std::ptrdiff_t>(byte_count)
                );
            merged_cloud->width += static_cast<uint32_t>(point_count);
        }
        merged_cloud->row_step = merged_cloud->width * merged_cloud->point_step;
        return merged_cloud;
    }

    // 高度过滤
    sensor_msgs::msg::PointCloud2 filter_cloud(const sensor_msgs::msg::PointCloud2 &cloud)
    {   
        // sensor_msgs::msg::PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_cloud;
        // 复制头部信息和属性
        filtered_cloud.header = cloud.header;
        filtered_cloud.height = 1;  // 无序点云的 height 为1
        filtered_cloud.fields = cloud.fields;   // 字段定义（x/y/z等字段的偏移量、类型，完全复用）
        filtered_cloud.is_bigendian = cloud.is_bigendian;
        filtered_cloud.point_step = cloud.point_step;
        filtered_cloud.row_step = 0;
        filtered_cloud.is_dense = false; // 是否包含无效点（false=允许有无效点，更通用）
        filtered_cloud.width = 0;
        filtered_cloud.data.clear();

        size_t z_offset = 0;
        if (!find_float32_field_offset(cloud, "z", z_offset))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Skipping PointCloud2 without FLOAT32 z field.");
            return filtered_cloud;
        }
        if (cloud.point_step == 0 || z_offset + sizeof(float) > cloud.point_step)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Skipping PointCloud2 with invalid point_step or z offset.");
            return filtered_cloud;
        }

        // 遍历所有的点， 过滤出高度适合的点
        const size_t point_count = valid_point_count(cloud);
        for (size_t i = 0; i < point_count; i++)
        {
            const size_t point_offset = i * cloud.point_step;
            if (point_offset + cloud.point_step > cloud.data.size())
            {
                break;
            }

            // 提取高度
            float z;
            memcpy(&z, cloud.data.data() + point_offset + z_offset, sizeof(float));

            // 只保留高度在 min_height_ 到 max_height_ 区间的点
            if(std::isfinite(z) && z >= min_height_ && z <= max_height_)
            {
                // 复制整个点的数据到过滤后点云
                filtered_cloud.data.insert(filtered_cloud.data.end(),
                        cloud.data.data() + point_offset,  // 当前点的 “字节块起始地址”
                        cloud.data.data() + point_offset + cloud.point_step // 当前点的 “字节块结束地址”
                    );
                filtered_cloud.width++;
            }
        } 
        filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;
        return filtered_cloud;
    }

    void timer_callback()
    {
        if(!accumulated_cloud_.data.empty())
        {
            // accumulated_cloud_.header.stamp = this->now();
            accumulated_cloud_.header.stamp = this->get_clock()->now();
            pub_->publish(accumulated_cloud_);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}
