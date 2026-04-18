#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <unordered_map>
#include <cmath>

struct PolarKey
{
    int r_bin;
    int theta_bin;
    bool operator==(const PolarKey & other) const
    {
        return r_bin == other.r_bin && theta_bin == other.theta_bin;
    }
};

struct PolarKeyHash
{
    size_t operator()(const PolarKey & k) const
    {
        size_t h = std::hash<int>()(k.r_bin);
        h ^= std::hash<int>()(k.theta_bin) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

class MbesIsm : public rclcpp::Node
{
public:
    MbesIsm() : Node("mbes_ism")
    {
        loadParams();
        setupRos();
    }

private:
    // params
    bool stonefish_;
    std::string pointcloud_topic_;
    std::string laserscan_topic_;
    std::string pointcloud_pub_topic_;
    float range_resolution_;  // metres per r bin

    // ros
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    void loadParams()
    {
        this->declare_parameter<bool>("stonefish", false);
        stonefish_ = this->get_parameter("stonefish").as_bool();

        this->declare_parameter<std::string>("pointcloud_topic", "/mbes/pointcloud");
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();

        this->declare_parameter<std::string>("laserscan_topic", "/mbes/laserscan");
        laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();

        this->declare_parameter<std::string>("pointcloud_pub_topic", "/mbes_ism/pointcloud");
        pointcloud_pub_topic_ = this->get_parameter("pointcloud_pub_topic").as_string();

        this->declare_parameter<double>("range_resolution", 1.0);
        range_resolution_ = static_cast<float>(this->get_parameter("range_resolution").as_double());
    }

    void setupRos()
    {
        if (stonefish_) {
            laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                laserscan_topic_, rclcpp::SensorDataQoS(),
                std::bind(&MbesIsm::laserscanCallback, this, std::placeholders::_1));
        } else {
            pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud_topic_, rclcpp::SensorDataQoS(),
                std::bind(&MbesIsm::pointcloudCallback, this, std::placeholders::_1));
        }

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_pub_topic_, 10);
    }


    void fillPolarGrid(std::unordered_map<PolarKey, float, PolarKeyHash> & polar_grid, float r, float theta)
    {
        int theta_bin = static_cast<int>(std::round(theta * 1e3f));
        int r_hit_bin = static_cast<int>(std::floor(r / range_resolution_));

        for (int rb = 0; rb < r_hit_bin; ++rb) {
            polar_grid[{rb, theta_bin}] = 0.1f;
        }
        polar_grid[{r_hit_bin, theta_bin}] = 0.9f;
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::unordered_map<PolarKey, float, PolarKeyHash> polar_grid;

        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_y != iter_y.end(); ++iter_y, ++iter_z) {
            float r     = std::sqrt(*iter_y * *iter_y + *iter_z * *iter_z);
            float theta = std::atan2(*iter_y, *iter_z);
            fillPolarGrid(polar_grid, r, theta);
        }

        pointcloud_pub_->publish(polarToPointCloud(polar_grid, msg->header));
    }

    void laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::unordered_map<PolarKey, float, PolarKeyHash> polar_grid;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
            float theta = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            fillPolarGrid(polar_grid, r, theta);
        }

        pointcloud_pub_->publish(polarToPointCloud(polar_grid, msg->header));
    }

    sensor_msgs::msg::PointCloud2 polarToPointCloud(
    const std::unordered_map<PolarKey, float, PolarKeyHash> & polar_grid,
    const std_msgs::msg::Header & header)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = header.frame_id;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(polar_grid.size());
        cloud.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(polar_grid.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_occ(cloud, "intensity");

        for (const auto & [key, occ] : polar_grid) {
            float r_c     = (key.r_bin + 0.5f) * range_resolution_;
            float theta_c = key.theta_bin * 1e-3f;
            if (stonefish_) {
                // Standard frame: range on x, cross-range on y, z=0
                *iter_x   = r_c * std::cos(theta_c);
                *iter_y   = r_c * std::sin(theta_c);
                *iter_z   = 0.0f;
            } else {
                // Real sensor frame: range on z, cross-range on y, x=0
                *iter_x   = 0.0f;
                *iter_y   = r_c * std::sin(theta_c);
                *iter_z   = r_c * std::cos(theta_c);
            }
            *iter_occ = occ;
            ++iter_x; ++iter_y; ++iter_z; ++iter_occ;
        }

        return cloud;
    }

};
