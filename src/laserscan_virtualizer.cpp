#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <memory>
#include <functional>
#include <chrono>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

class LaserscanVirtualizer : public rclcpp::Node
{
public:
    LaserscanVirtualizer();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_in);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, size_t pub_index);
    void virtual_laser_scan_parser();

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::TransformStamped> transforms_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> virtual_scan_publishers_;
    std::vector<std::string> output_frames_;

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double time_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;

    std::string cloud_frame_;
    std::string base_frame_;
    std::string cloud_topic_;
    std::string output_laser_topic_;
    std::string virtual_laser_scan_;
};

LaserscanVirtualizer::LaserscanVirtualizer()
    : Node("laserscan_virtualizer")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("base_frame", "cart_frame");
    this->declare_parameter<std::string>("cloud_topic", "/cloud_pcd");
    this->declare_parameter<std::string>("output_laser_topic", "");
    this->declare_parameter<std::string>("virtual_laser_scan", "");
    this->declare_parameter<double>("angle_min", -2.36);
    this->declare_parameter<double>("angle_max", 2.36);
    this->declare_parameter<double>("angle_increment", 0.0058);
    this->declare_parameter<double>("time_increment", 0.0);
    this->declare_parameter<double>("scan_time", 0.0333333);
    this->declare_parameter<double>("range_min", 0.45);
    this->declare_parameter<double>("range_max", 25.0);

    base_frame_ = this->get_parameter("base_frame").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    output_laser_topic_ = this->get_parameter("output_laser_topic").as_string();
    virtual_laser_scan_ = this->get_parameter("virtual_laser_scan").as_string();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    time_increment_ = this->get_parameter("time_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Parse virtual laser scan frames
    virtual_laser_scan_parser();

    // Subscribe to point cloud topic
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, 1,
        std::bind(&LaserscanVirtualizer::pointCloudCallback, this, _1));

    cloud_frame_ = "";
}

void LaserscanVirtualizer::virtual_laser_scan_parser()
{
    // Parse the virtual_laser_scan parameter to get frame names
    std::istringstream iss(virtual_laser_scan_);
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss),
              std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    std::vector<std::string> tmp_output_frames;

    for (const auto& token : tokens)
    {
        try
        {
            if (tf_buffer_->canTransform(base_frame_, token, tf2::TimePointZero, tf2::durationFromSec(1.0)))
            {
                RCLCPP_INFO(this->get_logger(), "Adding: %s", token.c_str());
                tmp_output_frames.push_back(token);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Can't transform: '%s' to '%s'", token.c_str(), base_frame_.c_str());
            }
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
        }
    }

    // Sort and remove duplicates
    std::sort(tmp_output_frames.begin(), tmp_output_frames.end());
    auto last = std::unique(tmp_output_frames.begin(), tmp_output_frames.end());
    tmp_output_frames.erase(last, tmp_output_frames.end());

    // Check if frames have changed
    if ((tmp_output_frames.size() != output_frames_.size()) ||
        !std::equal(tmp_output_frames.begin(), tmp_output_frames.end(), output_frames_.begin()))
    {
        // Clear previous publishers
        virtual_scan_publishers_.clear();
        cloud_frame_ = "";

        output_frames_ = tmp_output_frames;

        if (!output_frames_.empty())
        {
            virtual_scan_publishers_.resize(output_frames_.size());
            RCLCPP_INFO(this->get_logger(), "Publishing: %zu virtual scans", virtual_scan_publishers_.size());

            for (size_t i = 0; i < output_frames_.size(); ++i)
            {
                std::string topic_name;
                if (output_laser_topic_.empty())
                {
                    topic_name = output_frames_[i];
                }
                else
                {
                    topic_name = output_laser_topic_;
                }

                virtual_scan_publishers_[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 1);
                RCLCPP_INFO(this->get_logger(), "\t%s on topic %s", output_frames_[i].c_str(), topic_name.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Not publishing to any topic.");
        }
    }
}

void LaserscanVirtualizer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_in)
{
    if (cloud_frame_.empty())
    {
        cloud_frame_ = pcl_in->header.frame_id;
        transforms_.resize(output_frames_.size());

        for (size_t i = 0; i < output_frames_.size(); i++)
        {
            try
            {
                if (tf_buffer_->canTransform(output_frames_[i], cloud_frame_, tf2::TimePointZero, tf2::durationFromSec(2.0)))
                {
                    transforms_[i] = tf_buffer_->lookupTransform(output_frames_[i], cloud_frame_, tf2::TimePointZero);
                }
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
            }
        }
    }

    for (size_t i = 0; i < output_frames_.size(); i++)
    {
        PointCloudXYZ pcl_out, tmp_pcl;
        pcl::PCLPointCloud2 tmp_pcl2;

        // Convert ROS2 PointCloud2 to PCL
        pcl_conversions::toPCL(*pcl_in, tmp_pcl2);
        pcl::fromPCLPointCloud2(tmp_pcl2, tmp_pcl);

        // Initialize the header
        std::string tmp_frame = output_frames_[i];
        pcl_out.header = tmp_pcl.header;

        // Transform point cloud using TF2
        try
        {
            Eigen::Affine3d transform_eigen;
            transform_eigen = tf2::transformToEigen(transforms_[i]);
            pcl::transformPointCloud(tmp_pcl, pcl_out, transform_eigen.cast<float>());
            pcl_out.header.frame_id = tmp_frame;
        }
        catch (const std::exception& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            continue;
        }

        // Transform the PCL into Eigen matrix
        Eigen::MatrixXf tmp_eigen_matrix;
        pcl::toPCLPointCloud2(pcl_out, tmp_pcl2);
        pcl::getPointCloudAsEigen(tmp_pcl2, tmp_eigen_matrix);

        // Extract points and convert to laser scan
        pointcloud_to_laserscan(tmp_eigen_matrix, pcl_out.header, i);
    }
}

void LaserscanVirtualizer::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, size_t pub_index)
{
    auto output = std::make_unique<sensor_msgs::msg::LaserScan>();
    output->header = pcl_conversions::fromPCL(scan_header);
    output->angle_min = angle_min_;
    output->angle_max = angle_max_;
    output->angle_increment = angle_increment_;
    output->time_increment = time_increment_;
    output->scan_time = scan_time_;
    output->range_min = range_min_;
    output->range_max = range_max_;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for (int i = 0; i < points.cols(); i++)
    {
        const float& x = points(0, i);
        const float& y = points(1, i);
        const float& z = points(2, i);

        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)", x, y, z);
            continue;
        }

        double range_sq = y * y + x * x;
        double range_min_sq = output->range_min * output->range_min;
        if (range_sq < range_min_sq)
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                         range_sq, range_min_sq, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)",
                         angle, output->angle_min, output->angle_max);
            continue;
        }

        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
            output->ranges[index] = sqrt(range_sq);
    }

    virtual_scan_publishers_[pub_index]->publish(std::move(output));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserscanVirtualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
