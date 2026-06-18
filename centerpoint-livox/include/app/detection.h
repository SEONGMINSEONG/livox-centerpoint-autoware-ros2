#pragma once
#include <algorithm>
#include <queue>
#include <numeric>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ws_msgs/msg/cluster_array.hpp"
#include "ws_msgs/msg/bbox.hpp"
#include "ws_msgs/msg/bbox_array.hpp"
#include <pcl/filters/passthrough.h>
#include "centerpoint.h"
#include "process.h"
#include "postprocess.h"
#include "yaml-cpp/yaml.h"
#include <deque>
#include <chrono>

using Bbox = ws_msgs::msg::Bbox;
using bboxArray = ws_msgs::msg::BboxArray;
[[maybe_unused]] static constexpr size_t BoxFeature = 7;
using Clock = std::chrono::high_resolution_clock;
class Detection : public rclcpp::Node
{

public:
    Detection( const std::string& name_space,
        const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

    Detection(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());


private:
    void cloudCallbak(const sensor_msgs::msg::PointCloud2::ConstPtr &input);
    void makeOutput(std::vector<Box> &out_detections, const std_msgs::msg::Header& header);
    void publishMarkers(const bboxArray & bbox_array);
    std::vector<float> preprocessPoints(
        const float * points_array, int in_num_points, bool rotate_180, int & inside_model_range) const;
    float normalizeAngle(float angle) const;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    rclcpp::Publisher<ws_msgs::msg::BboxArray>::SharedPtr pub_bbox_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array_;

    rclcpp::Clock::SharedPtr clock_;

    std::unique_ptr<CenterPoint> centerpoint_ = nullptr;

    void loadParam(std::string & param_path);

    bboxArray bbox_array_;

    Clock::time_point m_sync_start_time_;

    bool use_onnx_ = false;
    std::string rpn_file_;
    std::string centerpoint_config_;
    std::string file_name_;
    std::string param_path_;
    std::string input_topic_ = "/livox/lidar";
    std::string output_topic_ = "bbox_array";
    std::string marker_topic_ = "/bbox_markers";
    std::string output_frame_id_;
    std::vector<double> point_cloud_range_ = {0.0, -44.8, -2.0, 224.0, 44.8, 4.0};
    double offset_ground_ = 0.0;
    double offset_angle_degrees_ = 0.0;
    bool undo_ground_offset_for_output_ = false;
    bool enable_rear_inference_ = true;
    bool publish_markers_ = true;
    double marker_lifetime_sec_ = 0.2;
    double marker_alpha_ = 0.45;
    int num_accumulated_frames_ = 2;
    int debug_every_n_frames_ = 20;
    std::deque<std::vector<float>> point_history_;

    int pub_count_ = 0;

    int sub_count_ = 0;

    std::vector<std::string> class_names_ = {"car","bicycle","pedestrians"};

};
