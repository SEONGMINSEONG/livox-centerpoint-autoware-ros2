#include "app/detection.h"
#include "ws_msgs/msg/bbox_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace std::chrono_literals;

namespace
{
std::string resolvePathRelativeToFile(const std::string & base_file, const std::string & path)
{
    if (path.empty() || path.front() == '/') {
        return path;
    }

    const auto slash = base_file.find_last_of('/');
    if (slash == std::string::npos) {
        return path;
    }
    return base_file.substr(0, slash + 1) + path;
}
}  // namespace

Detection::Detection(
    const rclcpp::NodeOptions& options
    ):Detection("",options) 
{}

Detection::Detection(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
    ):Node("Detection",name_space,options)
{
    RCLCPP_INFO(this->get_logger(),"Clustering init complete");

    // Store clock
    clock_ = this->get_clock();

    param_path_ = this->declare_parameter<std::string>("param_path", param_path_);
    if (param_path_.empty()) {
        param_path_ = ament_index_cpp::get_package_share_directory("centerpoint") + "/cfgs/centerpoint.yaml";
    }
    input_topic_ = this->declare_parameter<std::string>("input_topic", input_topic_);
    output_topic_ = this->declare_parameter<std::string>("output_topic", output_topic_);
    marker_topic_ = this->declare_parameter<std::string>("marker_topic", marker_topic_);
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", output_frame_id_);
    point_cloud_range_ = this->declare_parameter<std::vector<double>>("point_cloud_range", point_cloud_range_);
    offset_ground_ = this->declare_parameter<double>("offset_ground", offset_ground_);
    offset_angle_degrees_ = this->declare_parameter<double>("offset_angle_degrees", offset_angle_degrees_);
    undo_ground_offset_for_output_ =
        this->declare_parameter<bool>("undo_ground_offset_for_output", undo_ground_offset_for_output_);
    enable_rear_inference_ = this->declare_parameter<bool>("enable_rear_inference", enable_rear_inference_);
    publish_markers_ = this->declare_parameter<bool>("publish_markers", publish_markers_);
    marker_lifetime_sec_ = this->declare_parameter<double>("marker_lifetime_sec", marker_lifetime_sec_);
    marker_alpha_ = this->declare_parameter<double>("marker_alpha", marker_alpha_);
    num_accumulated_frames_ = this->declare_parameter<int>("num_accumulated_frames", num_accumulated_frames_);
    num_accumulated_frames_ = std::max(1, num_accumulated_frames_);
    debug_every_n_frames_ = this->declare_parameter<int>("debug_every_n_frames", debug_every_n_frames_);
    if (point_cloud_range_.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "point_cloud_range must have 6 values. Falling back to Livox default.");
        point_cloud_range_ = {0.0, -44.8, -2.0, 224.0, 44.8, 4.0};
    }
    if (num_accumulated_frames_ > 1) {
        RCLCPP_WARN(
            this->get_logger(),
            "Point accumulation has no ego-motion compensation. Use only for a stationary sensor test.");
    }

    loadParam(param_path_);

    // center_point_ 初始化
    centerpoint_.reset(new CenterPoint(use_onnx_,rpn_file_,centerpoint_config_));

    // Create a ROS subscriber for the input cloud
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> subscription_callback = std::bind(&Detection::cloudCallbak,this, std::placeholders::_1);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), subscription_callback
    );
    pub_bbox_array_ = this->create_publisher<ws_msgs::msg::BboxArray>(output_topic_, 10);
    if (publish_markers_) {
        pub_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    RCLCPP_INFO(this->get_logger(), "param_path: %s", param_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
    if (publish_markers_) {
        RCLCPP_INFO(this->get_logger(), "marker_topic: %s", marker_topic_.c_str());
    }
    RCLCPP_INFO(
        this->get_logger(),
        "centerpoint preprocessing: range=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] "
        "offset_ground=%.3f offset_angle_degrees=%.3f undo_for_output=%s enable_rear_inference=%s",
        point_cloud_range_[0], point_cloud_range_[1], point_cloud_range_[2],
        point_cloud_range_[3], point_cloud_range_[4], point_cloud_range_[5],
        offset_ground_, offset_angle_degrees_,
        undo_ground_offset_for_output_ ? "true" : "false",
        enable_rear_inference_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "num_accumulated_frames: %d", num_accumulated_frames_);
}

void Detection::loadParam(std::string & param_path)
{
    YAML::Node config = YAML::LoadFile(param_path);
    rpn_file_ = resolvePathRelativeToFile(param_path, config["RpnFile"].as<std::string>());
    centerpoint_config_ = resolvePathRelativeToFile(param_path, config["ModelConfig"].as<std::string>());
    use_onnx_ = config["UseOnnx"].as<bool>();
}

void Detection::cloudCallbak(const sensor_msgs::msg::PointCloud2::ConstPtr &input){
    std::cout<<"  ======================sub========================   ok     index "<< ++sub_count_ << std::endl;
        
    m_sync_start_time_ = Clock::now();
    RCLCPP_INFO(this->get_logger(),"points_size(%d,%d)",input->height,input->width);
    PointICloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    float *points_array;
    int in_num_points = PointCloud2Array(points_array,*cloud);
    std::vector<float> current_points;
    if (in_num_points > 0) {
        current_points.assign(points_array, points_array + static_cast<size_t>(in_num_points) * 4U);
    }
    delete[] points_array;

    point_history_.push_back(std::move(current_points));
    while (point_history_.size() > static_cast<size_t>(num_accumulated_frames_)) {
        point_history_.pop_front();
    }

    size_t accumulated_values = 0;
    for (const auto & frame_points : point_history_) {
        accumulated_values += frame_points.size();
    }
    std::vector<float> accumulated_points;
    accumulated_points.reserve(accumulated_values);
    for (const auto & frame_points : point_history_) {
        accumulated_points.insert(accumulated_points.end(), frame_points.begin(), frame_points.end());
    }
    const int accumulated_num_points = static_cast<int>(accumulated_points.size() / 4U);
    const float * accumulated_data = accumulated_points.data();

    int front_points_count = 0;
    auto front_points = preprocessPoints(accumulated_data, accumulated_num_points, false, front_points_count);
    const bool should_log_debug =
        (debug_every_n_frames_ > 0) && ((sub_count_ - 1) % debug_every_n_frames_) == 0;
    if (should_log_debug && accumulated_num_points > 0) {
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();
        int inside_model_range = 0;
        for (int i = 0; i < accumulated_num_points; ++i) {
            const float x = accumulated_data[4 * i];
            const float y = accumulated_data[4 * i + 1];
            const float z = accumulated_data[4 * i + 2] +
                static_cast<float>(x * std::tan(offset_angle_degrees_ * M_PI / 180.0) + offset_ground_);
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            min_z = std::min(min_z, z);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
            max_z = std::max(max_z, z);
            if (x > point_cloud_range_[0] && x < (point_cloud_range_[3] - 0.01) &&
                y > point_cloud_range_[1] && y < (point_cloud_range_[4] - 0.01) &&
                z > point_cloud_range_[2] && z < (point_cloud_range_[5] - 0.01)) {
                ++inside_model_range;
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "input stats frame=%s raw_points=%d accumulated_points=%d front_points=%d inside_model_range=%d "
            "x=[%.2f, %.2f] y=[%.2f, %.2f] corrected_z=[%.2f, %.2f]",
            input->header.frame_id.c_str(), in_num_points, accumulated_num_points,
            front_points_count, inside_model_range,
            min_x, max_x, min_y, max_y, min_z, max_z);
    }
    std::vector<Box> out_detections;
    out_detections.clear();
    if (front_points_count > 0) {
        cudaDeviceSynchronize();
        centerpoint_->DoInference(front_points.data(), front_points_count, out_detections);
        cudaDeviceSynchronize();
    }

    int rear_points_count = 0;
    if (enable_rear_inference_) {
        auto rear_points = preprocessPoints(accumulated_data, accumulated_num_points, true, rear_points_count);
        if (rear_points_count > 0) {
            std::vector<Box> rear_detections;
            cudaDeviceSynchronize();
            centerpoint_->DoInference(rear_points.data(), rear_points_count, rear_detections);
            cudaDeviceSynchronize();
            for (auto & box : rear_detections) {
                box.x *= -1.0F;
                box.y *= -1.0F;
                box.r = normalizeAngle(box.r + static_cast<float>(M_PI));
                out_detections.push_back(box);
            }
        }
    }

    if (should_log_debug) {
        RCLCPP_INFO(
            this->get_logger(),
            "centerpoint preprocessed frame=%d raw=%d accumulated=%d front=%d rear=%d detections=%zu",
            sub_count_, in_num_points, accumulated_num_points,
            front_points_count, rear_points_count, out_detections.size());
    }
    makeOutput(out_detections, input->header);
    double sync_duration_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - m_sync_start_time_).count() / 1e6;
    std::cout<<" --------------cloudCallbak-------------------------- :   "<<sync_duration_ms<<std::endl;
}

std::vector<float> Detection::preprocessPoints(
    const float * points_array, int in_num_points, bool rotate_180, int & inside_model_range) const
{
    std::vector<float> output;
    output.reserve(static_cast<size_t>(std::max(in_num_points, 0)) * 4U);
    inside_model_range = 0;

    const float min_x = static_cast<float>(point_cloud_range_[0]);
    const float min_y = static_cast<float>(point_cloud_range_[1]);
    const float min_z = static_cast<float>(point_cloud_range_[2]);
    const float max_x = static_cast<float>(point_cloud_range_[3] - 0.01);
    const float max_y = static_cast<float>(point_cloud_range_[4] - 0.01);
    const float max_z = static_cast<float>(point_cloud_range_[5] - 0.01);
    const float angle_tan = static_cast<float>(std::tan(offset_angle_degrees_ * M_PI / 180.0));

    for (int i = 0; i < in_num_points; ++i) {
        float x = points_array[4 * i];
        float y = points_array[4 * i + 1];
        float z = points_array[4 * i + 2];
        const float intensity = points_array[4 * i + 3];
        if (rotate_180) {
            x *= -1.0F;
            y *= -1.0F;
        }

        z += static_cast<float>(x * angle_tan + offset_ground_);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity)) {
            continue;
        }
        if (x <= min_x || x >= max_x || y <= min_y || y >= max_y || z <= min_z || z >= max_z) {
            continue;
        }

        output.push_back(x);
        output.push_back(y);
        output.push_back(z);
        output.push_back(intensity);
        ++inside_model_range;
    }
    return output;
}

float Detection::normalizeAngle(float angle) const
{
    while (angle > static_cast<float>(M_PI)) {
        angle -= static_cast<float>(2.0 * M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
        angle += static_cast<float>(2.0 * M_PI);
    }
    return angle;
}

void Detection::makeOutput(std::vector<Box> &out_detections, const std_msgs::msg::Header& header)
{
    size_t num_objects = out_detections.size();
    bbox_array_.header = header;
    if (!output_frame_id_.empty()) {
        bbox_array_.header.frame_id = output_frame_id_;
    }
    bbox_array_.source_type = 2;
    bbox_array_.boxes.clear();
    for(size_t obj_index = 0; obj_index < num_objects; obj_index++) {
        ws_msgs::msg::Bbox bbox;
        float x = out_detections[obj_index].x;
        float y = out_detections[obj_index].y;
        float z = out_detections[obj_index].z - 1.8;
        if (undo_ground_offset_for_output_) {
            z -= static_cast<float>(x * std::tan(offset_angle_degrees_ * M_PI / 180.0) + offset_ground_);
        }
        float dx = out_detections[obj_index].l;
        float dy = out_detections[obj_index].w;
        float dz = out_detections[obj_index].h;
        float yaw = out_detections[obj_index].r;
        yaw = std::atan2(sinf(yaw),cosf(yaw));
        // yaw = - yaw;

        float top_z = z + dz ;
        float bot_Z = z;     
        float c_s = cos(yaw);
        float s_s = sin(yaw);
        bbox.center.x = x;
        bbox.center.y = y;
        bbox.center.z = z;
        bbox.size.x = dy;
        bbox.size.y = dx;
        bbox.size.z = dz;
        bbox.heading = yaw;
        bbox.type = out_detections[obj_index].label;
        bbox.existence_probability = out_detections[obj_index].score;
        // p1 上右前
        bbox.corner_3d[0].x = dx/2*c_s - dy /2 *s_s + x;
        bbox.corner_3d[0].y = dy /2 * c_s + dx / 2 * s_s + y;
        bbox.corner_3d[0].z = top_z;
        // p2  上左前
        bbox.corner_3d[1].x = (-dx)/2*c_s - dy /2 *s_s + x;
        bbox.corner_3d[1].y = dy /2 * c_s + (-dx) / 2 * s_s + y;
        bbox.corner_3d[1].z = top_z;
        // p3  上左后
        bbox.corner_3d[2].x = (-dx)/2*c_s - (-dy) /2 *s_s + x;
        bbox.corner_3d[2].y = (-dy) /2 * c_s + (-dx) / 2 * s_s + y;
        bbox.corner_3d[2].z = top_z;
        // p4   上右后
        bbox.corner_3d[3].x = dx/2*c_s - (-dy) /2 *s_s + x;
        bbox.corner_3d[3].y = (-dy) /2 * c_s + dx / 2 * s_s + y;
        bbox.corner_3d[3].z = top_z;
        // p5   下右前
        bbox.corner_3d[4].x = dx/2*c_s - dy /2 *s_s + x;
        bbox.corner_3d[4].y = dy /2 * c_s + dx / 2 * s_s + y;
        bbox.corner_3d[4].z = bot_Z;
        // p6   下左前
        bbox.corner_3d[5].x = (-dx)/2*c_s - dy /2 *s_s + x;
        bbox.corner_3d[5].y = dy /2 * c_s + (-dx) / 2 * s_s + y;
        bbox.corner_3d[5].z = bot_Z;
        // p7   下左后
        bbox.corner_3d[6].x = (-dx)/2*c_s - (-dy) /2 *s_s + x;
        bbox.corner_3d[6].y = (-dy) /2 * c_s + (-dx) / 2 * s_s + y;
        bbox.corner_3d[6].z = bot_Z;
        // p8   下右后
        bbox.corner_3d[7].x = dx/2*c_s - (-dy) /2 *s_s + x;
        bbox.corner_3d[7].y = (-dy) /2 * c_s + dx / 2 * s_s + y;
        bbox.corner_3d[7].z = bot_Z;

        bbox_array_.boxes.push_back(bbox);
    }
    pub_bbox_array_->publish(bbox_array_);
    if (publish_markers_) {
        publishMarkers(bbox_array_);
    }
    std::cout<<"  ------------publish   ok     index--------- "<< ++pub_count_<<"   detected objects:  "<<num_objects<<std::endl;
}

void Detection::publishMarkers(const bboxArray & bbox_array)
{
    if (!pub_marker_array_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = bbox_array.header;
    clear_marker.ns = "centerpoint_bbox";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 0;
    for (const auto & bbox : bbox_array.boxes) {
        visualization_msgs::msg::Marker marker;
        marker.header = bbox_array.header;
        marker.ns = "centerpoint_bbox";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        const double yaw = static_cast<double>(bbox.heading);
        const double height = std::max(0.05F, std::abs(bbox.size.z));
        marker.pose.position.x = bbox.center.x;
        marker.pose.position.y = bbox.center.y;
        marker.pose.position.z = bbox.center.z + height * 0.5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(yaw * 0.5);
        marker.pose.orientation.w = std::cos(yaw * 0.5);

        marker.scale.x = std::max(0.05F, std::abs(bbox.size.y));
        marker.scale.y = std::max(0.05F, std::abs(bbox.size.x));
        marker.scale.z = height;

        marker.color.a = static_cast<float>(std::max(0.0, std::min(1.0, marker_alpha_)));
        if (bbox.type == 1) {
            marker.color.r = 0.1F;
            marker.color.g = 0.85F;
            marker.color.b = 0.25F;
        } else if (bbox.type == 2) {
            marker.color.r = 1.0F;
            marker.color.g = 0.35F;
            marker.color.b = 0.55F;
        } else {
            marker.color.r = 0.1F;
            marker.color.g = 0.45F;
            marker.color.b = 1.0F;
        }

        if (marker_lifetime_sec_ > 0.0) {
            const double sec_floor = std::floor(marker_lifetime_sec_);
            marker.lifetime.sec = static_cast<int32_t>(sec_floor);
            marker.lifetime.nanosec =
                static_cast<uint32_t>((marker_lifetime_sec_ - sec_floor) * 1000000000.0);
        }
        marker_array.markers.push_back(marker);
    }

    pub_marker_array_->publish(marker_array);
}
