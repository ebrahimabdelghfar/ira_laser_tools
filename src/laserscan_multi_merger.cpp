#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <memory>
#include <functional>
#include <chrono>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/exceptions.h>

#include <laser_geometry/laser_geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, const std::string& topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2* merged_cloud);
    void laserscan_topic_parser();
    void topic_parser_timer_callback();

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    laser_geometry::LaserProjection projector_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers_;
    std::vector<bool> clouds_modified_;

    std::vector<pcl::PCLPointCloud2> clouds_;
    std::vector<std::string> input_topics_;

    rclcpp::TimerBase::SharedPtr topic_parser_timer_;

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double time_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;

    std::string destination_frame_;
    std::string cloud_destination_topic_;
    std::string scan_destination_topic_;
    std::string laserscan_topics_;
};

LaserscanMerger::LaserscanMerger()
    : Node("laserscan_multi_merger")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("destination_frame", "cart_frame");
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
    this->declare_parameter<std::string>("laserscan_topics", "");
    this->declare_parameter<double>("angle_min", -2.36);
    this->declare_parameter<double>("angle_max", 2.36);
    this->declare_parameter<double>("angle_increment", 0.0058);
    this->declare_parameter<double>("time_increment", 0.0);
    this->declare_parameter<double>("scan_time", 0.0333333);
    this->declare_parameter<double>("range_min", 0.45);
    this->declare_parameter<double>("range_max", 25.0);

    destination_frame_ = this->get_parameter("destination_frame").as_string();
    cloud_destination_topic_ = this->get_parameter("cloud_destination_topic").as_string();
    scan_destination_topic_ = this->get_parameter("scan_destination_topic").as_string();
    laserscan_topics_ = this->get_parameter("laserscan_topics").as_string();
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

    // Create publishers
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_destination_topic_, 1);
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        scan_destination_topic_, 1);

    // Parse laser scan topics and subscribe
    laserscan_topic_parser();

    // Create a timer to periodically check for new topics
    topic_parser_timer_ = this->create_wall_timer(
        1s, std::bind(&LaserscanMerger::topic_parser_timer_callback, this));
}

void LaserscanMerger::topic_parser_timer_callback()
{
    laserscan_topic_parser();
}

void LaserscanMerger::laserscan_topic_parser()
{
    // Parse the laserscan_topics parameter to get topic names
    std::istringstream iss(laserscan_topics_);
    std::set<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss),
              std::istream_iterator<std::string>(),
              std::inserter(tokens, tokens.begin()));

    // Get list of available topics
    auto topics_and_types = this->get_topic_names_and_types();

    std::vector<std::string> tmp_input_topics;

    for (const auto& topic_pair : topics_and_types)
    {
        const std::string& topic_name = topic_pair.first;
        const std::vector<std::string>& topic_types = topic_pair.second;

        for (const auto& topic_type : topic_types)
        {
            if (topic_type == "sensor_msgs/msg/LaserScan" && tokens.count(topic_name) > 0)
            {
                tmp_input_topics.push_back(topic_name);
                tokens.erase(topic_name);
            }
        }
    }

    // Sort and remove duplicates
    std::sort(tmp_input_topics.begin(), tmp_input_topics.end());
    auto last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(last, tmp_input_topics.end());

    // Check if topics have changed
    if ((tmp_input_topics.size() != input_topics_.size()) ||
        !std::equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics_.begin()))
    {
        // Clear previous subscribers
        scan_subscribers_.clear();

        input_topics_ = tmp_input_topics;

        if (!input_topics_.empty())
        {
            scan_subscribers_.resize(input_topics_.size());
            clouds_modified_.resize(input_topics_.size());
            clouds_.resize(input_topics_.size());

            RCLCPP_INFO(this->get_logger(), "Subscribing to %zu topics", scan_subscribers_.size());

            for (size_t i = 0; i < input_topics_.size(); ++i)
            {
                scan_subscribers_[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    input_topics_[i], 1,
                    [this, topic = input_topics_[i]](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
                        this->scanCallback(scan, topic);
                    });
                clouds_modified_[i] = false;
                RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topics_[i].c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Not subscribed to any topic.");
        }
    }
}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, const std::string& topic)
{
    sensor_msgs::msg::PointCloud2 tmp_cloud;

    try
    {
        // Wait for transform
        if (!tf_buffer_->canTransform(destination_frame_, scan->header.frame_id,
                                       tf2::TimePointZero, tf2::durationFromSec(1.0)))
        {
            RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s",
                        scan->header.frame_id.c_str(), destination_frame_.c_str());
            return;
        }

        // Project laser scan to point cloud and transform to destination frame
        projector_.transformLaserScanToPointCloud(destination_frame_, *scan, tmp_cloud, *tf_buffer_);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
        return;
    }

    // Find which topic this scan belongs to and store the cloud
    for (size_t i = 0; i < input_topics_.size(); i++)
    {
        if (topic == input_topics_[i])
        {
            pcl_conversions::toPCL(tmp_cloud, clouds_[i]);
            clouds_modified_[i] = true;
        }
    }

    // Count how many scans we have received
    int total_clouds = 0;
    for (size_t i = 0; i < clouds_modified_.size(); i++)
    {
        if (clouds_modified_[i])
            total_clouds++;
    }

    // Proceed only if all subscribed scans have arrived
    if (static_cast<size_t>(total_clouds) == clouds_modified_.size())
    {
        pcl::PCLPointCloud2 merged_cloud = clouds_[0];
        clouds_modified_[0] = false;

        for (size_t i = 1; i < clouds_modified_.size(); i++)
        {
            pcl::concatenate(merged_cloud, clouds_[i], merged_cloud);
            clouds_modified_[i] = false;
        }

        // Publish merged point cloud
        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl_conversions::fromPCL(merged_cloud, output_cloud);
        point_cloud_publisher_->publish(output_cloud);

        // Convert to laser scan
        Eigen::MatrixXf points;
        pcl::getPointCloudAsEigen(merged_cloud, points);
        pointcloud_to_laserscan(points, &merged_cloud);
    }
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2* merged_cloud)
{
    auto output = std::make_unique<sensor_msgs::msg::LaserScan>();
    output->header = pcl_conversions::fromPCL(merged_cloud->header);
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

    laser_scan_publisher_->publish(std::move(output));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserscanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
