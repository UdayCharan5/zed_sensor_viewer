// zed_sensor_rviz.cpp — ZED Sensor & Positional Tracking Visualization in RViz

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sl/Camera.hpp>

#include <vector>
#include <chrono>
#include <memory>
#include <cmath>

inline sl::Translation rotateVector(const sl::Orientation& q, float vx, float vy, float vz) {
    float qx = q.ox, qy = q.oy, qz = q.oz, qw = q.ow;
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);
    float rx = vx + qw * tx + (qy * tz - qz * ty);
    float ry = vy + qw * ty + (qz * tx - qx * tz);
    float rz = vz + qw * tz + (qx * ty - qy * tx);
    return sl::Translation(rx, ry, rz);
}

inline sl::Translation transformPoint(const sl::Translation& local_pt, const sl::Translation& cam_trans, const sl::Orientation& cam_orient) {
    sl::Translation rotated = rotateVector(cam_orient, local_pt.x, local_pt.y, local_pt.z);
    return sl::Translation(cam_trans.x + rotated.x, cam_trans.y + rotated.y, cam_trans.z + rotated.z);
}

class ZedSensorRvizNode : public rclcpp::Node {
public:
    bool init_ok = false;

    ZedSensorRvizNode() : Node("zed_sensor_rviz") {
        pose_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("zed/pose", 10);
        path_pub_   = create_publisher<nav_msgs::msg::Path>("zed/path", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("zed/sensor_markers", 10);
        imu_pub_    = create_publisher<sensor_msgs::msg::Imu>("zed/imu", 10);

        sl::InitParameters ip;
        ip.camera_resolution = sl::RESOLUTION::HD720;
        ip.camera_fps = 30;
        ip.depth_mode = sl::DEPTH_MODE::PERFORMANCE; 
        ip.coordinate_units = sl::UNIT::METER;
        ip.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

        sl::ERROR_CODE open_err = zed_.open(ip);
        if (open_err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "ZED Camera failed to open: %s", sl::toString(open_err).c_str());
            return;
        }

        sl::PositionalTrackingParameters tp;
        tp.enable_area_memory = true; // Loop closure enabled
        tp.set_gravity_as_origin = true; // Anchors the Z-axis strictly to the IMU gravity vector
        tp.enable_pose_smoothing = true; // Smooths out violent positional jumps
        
        sl::ERROR_CODE track_err = zed_.enablePositionalTracking(tp);
        if (track_err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Tracking failed to enable: %s", sl::toString(track_err).c_str());
            zed_.close();
            return;
        }

        has_imu_ = zed_.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER);
        if (has_imu_) {
            RCLCPP_INFO(this->get_logger(), "ZED IMU detected. IMPORTANT: Keep the camera perfectly still for 3 seconds to calibrate IMU biases and prevent drift.");
        }

        path_msg_.header.frame_id = "odom";

        timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ZedSensorRvizNode::tick, this)
        );

        init_ok = true;
    }

    ~ZedSensorRvizNode() {
        if (zed_.isOpened()) {
            zed_.disablePositionalTracking();
            zed_.close();
        }
    }

private:
    void tick() {
        if (zed_.grab() != sl::ERROR_CODE::SUCCESS) return;

        rclcpp::Time now = this->get_clock()->now();

        // 1. Get Camera Pose
        sl::Pose camera_pose;
        zed_.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);

        sl::Translation t = camera_pose.getTranslation();
        sl::Orientation q = camera_pose.getOrientation();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = t.x;
        pose_msg.pose.position.y = t.y;
        pose_msg.pose.position.z = t.z;
        pose_msg.pose.orientation.x = q.ox;
        pose_msg.pose.orientation.y = q.oy;
        pose_msg.pose.orientation.z = q.oz;
        pose_msg.pose.orientation.w = q.ow;
        pose_pub_->publish(pose_msg);

        path_msg_.header.stamp = now;
        path_msg_.poses.push_back(pose_msg);
        if (path_msg_.poses.size() > 2000) path_msg_.poses.erase(path_msg_.poses.begin());
        path_pub_->publish(path_msg_);

        // 2. Fetch & Filter Sensor Data
        sl::SensorsData sensors_data;
        sl::Translation acc_local(0.0f, 0.0f, 0.0f);
        sl::Translation acc_world(0.0f, 0.0f, 0.0f);

        if (has_imu_) {
            zed_.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);
            auto imu_data = sensors_data.imu;

            // Apply Exponential Moving Average (EMA) Low-Pass Filter
            if (first_imu_read_) {
                filtered_acc_local_.x = imu_data.linear_acceleration.x;
                filtered_acc_local_.y = imu_data.linear_acceleration.y;
                filtered_acc_local_.z = imu_data.linear_acceleration.z;
                filtered_gyro_local_.x = imu_data.angular_velocity.x;
                filtered_gyro_local_.y = imu_data.angular_velocity.y;
                filtered_gyro_local_.z = imu_data.angular_velocity.z;
                first_imu_read_ = false;
            } else {
                filtered_acc_local_.x = ema_alpha_ * imu_data.linear_acceleration.x + (1.0f - ema_alpha_) * filtered_acc_local_.x;
                filtered_acc_local_.y = ema_alpha_ * imu_data.linear_acceleration.y + (1.0f - ema_alpha_) * filtered_acc_local_.y;
                filtered_acc_local_.z = ema_alpha_ * imu_data.linear_acceleration.z + (1.0f - ema_alpha_) * filtered_acc_local_.z;
                
                filtered_gyro_local_.x = ema_alpha_ * imu_data.angular_velocity.x + (1.0f - ema_alpha_) * filtered_gyro_local_.x;
                filtered_gyro_local_.y = ema_alpha_ * imu_data.angular_velocity.y + (1.0f - ema_alpha_) * filtered_gyro_local_.y;
                filtered_gyro_local_.z = ema_alpha_ * imu_data.angular_velocity.z + (1.0f - ema_alpha_) * filtered_gyro_local_.z;
            }

            acc_world = rotateVector(q, filtered_acc_local_.x, filtered_acc_local_.y, filtered_acc_local_.z);

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = "zed_camera_link";
            imu_msg.orientation = pose_msg.pose.orientation;
            imu_msg.linear_acceleration.x = filtered_acc_local_.x;
            imu_msg.linear_acceleration.y = filtered_acc_local_.y;
            imu_msg.linear_acceleration.z = filtered_acc_local_.z;
            imu_msg.angular_velocity.x = filtered_gyro_local_.x * M_PI / 180.0; 
            imu_msg.angular_velocity.y = filtered_gyro_local_.y * M_PI / 180.0;
            imu_msg.angular_velocity.z = filtered_gyro_local_.z * M_PI / 180.0;
            imu_pub_->publish(imu_msg);
        }

        // 3. Populate MarkerArray for RViz
        visualization_msgs::msg::MarkerArray ma;

        // (Grid, Body, Lenses, and Facing markers remain identical... keeping concise here)
        // ... [Insert Marker 1 to 5 exactly as they were in your original code] ...

        // 4. Draw Smoothed Accelerometer Vectors
        if (has_imu_) {
            const float acc_scale = 0.035f;

            geometry_msgs::msg::Point p_start;
            p_start.x = t.x; p_start.y = t.y; p_start.z = t.z;

            // Raw Acceleration (contains gravity, now smoothed)
            visualization_msgs::msg::Marker raw_acc_marker;
            raw_acc_marker.header.stamp = now;
            raw_acc_marker.header.frame_id = "odom";
            raw_acc_marker.ns = "acceleration_vectors";
            raw_acc_marker.id = 5;
            raw_acc_marker.type = visualization_msgs::msg::Marker::ARROW;
            raw_acc_marker.action = visualization_msgs::msg::Marker::ADD;
            raw_acc_marker.scale.x = 0.012; raw_acc_marker.scale.y = 0.028; raw_acc_marker.scale.z = 0.040;
            raw_acc_marker.color.r = 1.0f; raw_acc_marker.color.g = 0.1f; raw_acc_marker.color.b = 0.1f; raw_acc_marker.color.a = 0.85f;

            geometry_msgs::msg::Point p_raw_end;
            p_raw_end.x = t.x + acc_world.x * acc_scale;
            p_raw_end.y = t.y + acc_world.y * acc_scale;
            p_raw_end.z = t.z + acc_world.z * acc_scale;

            raw_acc_marker.points.push_back(p_start);
            raw_acc_marker.points.push_back(p_raw_end);
            ma.markers.push_back(raw_acc_marker);

            // Gravity Marker (Downwards)
            visualization_msgs::msg::Marker gravity_marker;
            gravity_marker.header.stamp = now;
            gravity_marker.header.frame_id = "odom";
            gravity_marker.ns = "acceleration_vectors";
            gravity_marker.id = 6;
            gravity_marker.type = visualization_msgs::msg::Marker::ARROW;
            gravity_marker.action = visualization_msgs::msg::Marker::ADD;
            gravity_marker.scale.x = 0.012; gravity_marker.scale.y = 0.028; gravity_marker.scale.z = 0.040;
            gravity_marker.color.r = 0.1f; gravity_marker.color.g = 1.0f; gravity_marker.color.b = 0.1f; gravity_marker.color.a = 0.85f;

            geometry_msgs::msg::Point p_grav_end;
            p_grav_end.x = t.x; p_grav_end.y = t.y; p_grav_end.z = t.z - 9.81f * acc_scale;

            gravity_marker.points.push_back(p_start);
            gravity_marker.points.push_back(p_grav_end);
            ma.markers.push_back(gravity_marker);

            // Dynamic Linear Acceleration (Motion only, much cleaner now)
            visualization_msgs::msg::Marker dynamic_acc_marker;
            dynamic_acc_marker.header.stamp = now;
            dynamic_acc_marker.header.frame_id = "odom";
            dynamic_acc_marker.ns = "acceleration_vectors";
            dynamic_acc_marker.id = 7;
            dynamic_acc_marker.type = visualization_msgs::msg::Marker::ARROW;
            dynamic_acc_marker.action = visualization_msgs::msg::Marker::ADD;
            dynamic_acc_marker.scale.x = 0.016; dynamic_acc_marker.scale.y = 0.034; dynamic_acc_marker.scale.z = 0.045;
            dynamic_acc_marker.color.r = 0.0f; dynamic_acc_marker.color.g = 0.85f; dynamic_acc_marker.color.b = 1.0f; dynamic_acc_marker.color.a = 0.95f;

            sl::Translation dyn_acc_world(acc_world.x, acc_world.y, acc_world.z - 9.81f);
            float dyn_mag = std::sqrt(dyn_acc_world.x * dyn_acc_world.x + dyn_acc_world.y * dyn_acc_world.y + dyn_acc_world.z * dyn_acc_world.z);
            
            if (dyn_mag > 0.10f) { // Lowered threshold slightly since data is less noisy
                geometry_msgs::msg::Point p_dyn_end;
                p_dyn_end.x = t.x + dyn_acc_world.x * acc_scale;
                p_dyn_end.y = t.y + dyn_acc_world.y * acc_scale;
                p_dyn_end.z = t.z + dyn_acc_world.z * acc_scale;

                dynamic_acc_marker.points.push_back(p_start);
                dynamic_acc_marker.points.push_back(p_dyn_end);
                ma.markers.push_back(dynamic_acc_marker);
            }
        }

        marker_pub_->publish(ma);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sl::Camera zed_;
    bool has_imu_ = false;
    nav_msgs::msg::Path path_msg_;

    // EMA Filter Variables
    bool first_imu_read_ = true;
    const float ema_alpha_ = 0.15f; // Adjust between 0.0 (no update) and 1.0 (raw noise)
    sl::Translation filtered_acc_local_{0.0f, 0.0f, 0.0f};
    sl::Translation filtered_gyro_local_{0.0f, 0.0f, 0.0f};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedSensorRvizNode>();
    if (node->init_ok) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}