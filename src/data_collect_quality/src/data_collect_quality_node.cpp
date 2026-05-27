#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "weld_interface/msg/collection_quality.hpp"
#include "weld_interface/msg/data_collect_status.hpp"
#include "weld_interface/topic_configs.h"
#include "acquisition_interfaces/msg/acquisition_quality.hpp"

class DataCollectQualityNode : public rclcpp::Node {
public:
    DataCollectQualityNode() : Node("data_collect_quality_node") {
        expected_image_fps_ = this->declare_parameter<double>("expected_image_fps", 15.0);
        min_blur_variance_ = this->declare_parameter<double>("min_blur_variance", 120.0);
        expected_points_per_cloud_ = this->declare_parameter<double>("expected_points_per_cloud", 40000.0);
        warn_sync_ms_ = this->declare_parameter<double>("warn_sync_ms", 50.0);
        fail_sync_ms_ = this->declare_parameter<double>("fail_sync_ms", 100.0);
        warn_frame_loss_rate_ = this->declare_parameter<double>("warn_frame_loss_rate", 0.05);
        fail_frame_loss_rate_ = this->declare_parameter<double>("fail_frame_loss_rate", 0.10);
        warn_blur_score_ = this->declare_parameter<double>("warn_blur_score", 70.0);
        fail_blur_score_ = this->declare_parameter<double>("fail_blur_score", 40.0);
        warn_cloud_completeness_ = this->declare_parameter<double>("warn_cloud_completeness", 70.0);
        fail_cloud_completeness_ = this->declare_parameter<double>("fail_cloud_completeness", 40.0);

        sub_status_ = this->create_subscription<weld_interface::msg::DataCollectStatus>(
            DATA_COLLECT_STATUS_TOPIC_NAME,
            10,
            std::bind(&DataCollectQualityNode::on_status, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC_NAME,
            10,
            std::bind(&DataCollectQualityNode::on_image, this, std::placeholders::_1));

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            POINT_CLOUD_TOPIC_NAME,
            10,
            std::bind(&DataCollectQualityNode::on_cloud, this, std::placeholders::_1));

        pub_quality_ = this->create_publisher<weld_interface::msg::CollectionQuality>(
            DATA_COLLECT_QUALITY_TOPIC_NAME,
            rclcpp::QoS(1).transient_local());
        pub_acquisition_quality_ = this->create_publisher<acquisition_interfaces::msg::AcquisitionQuality>(
            "/acquisition/quality",
            rclcpp::QoS(1).transient_local());

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DataCollectQualityNode::publish_quality, this));

        RCLCPP_INFO(this->get_logger(), "[DataCollectQuality] node started.");
    }

private:
    void reset_session_stats() {
        image_count_ = 0;
        point_cloud_count_ = 0;
        blur_sum_ = 0.0;
        blur_samples_ = 0;
        sync_sum_ms_ = 0.0;
        sync_samples_ = 0;
        point_cloud_completeness_sum_ = 0.0;
        has_last_image_stamp_ = false;
        has_last_cloud_stamp_ = false;
        has_first_image_stamp_ = false;
        has_last_image_for_span_ = false;
    }

    void on_status(const weld_interface::msg::DataCollectStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (msg == nullptr) {
            return;
        }

        if (msg->running && !running_) {
            running_ = true;
            session_dir_ = msg->current_save_dir;
            reset_session_stats();
            reason_ = "collecting";
            return;
        }

        if (!msg->running && running_) {
            running_ = false;
            reason_ = "collection_stopped";
            return;
        }

        session_dir_ = msg->current_save_dir;
    }

    void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || msg == nullptr) {
            return;
        }

        const rclcpp::Time image_stamp(msg->header.stamp);
        if (!has_first_image_stamp_) {
            first_image_stamp_ = image_stamp;
            has_first_image_stamp_ = true;
        }
        last_image_stamp_for_span_ = image_stamp;
        has_last_image_for_span_ = true;

        if (has_last_image_stamp_) {
            const double dt = (image_stamp - last_image_stamp_).seconds();
            (void)dt;
        } else {
            has_last_image_stamp_ = true;
        }
        ++image_count_;
        last_image_stamp_ = image_stamp;

        if (has_last_cloud_stamp_) {
            const double diff_ms = std::abs((image_stamp - last_cloud_stamp_).seconds() * 1000.0);
            sync_sum_ms_ += diff_ms;
            ++sync_samples_;
        }

        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat gray;
            if (cv_ptr->image.channels() == 1) {
                gray = cv_ptr->image;
            } else {
                cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
            }

            cv::Mat lap;
            cv::Laplacian(gray, lap, CV_64F);
            cv::Scalar mu;
            cv::Scalar sigma;
            cv::meanStdDev(lap, mu, sigma);
            const double variance = sigma.val[0] * sigma.val[0];
            blur_sum_ += variance;
            ++blur_samples_;
        } catch (const std::exception& e) {
            reason_ = std::string("image_convert_failed: ") + e.what();
        }
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || msg == nullptr) {
            return;
        }

        last_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
        has_last_cloud_stamp_ = true;

        const double points = static_cast<double>(msg->width) * static_cast<double>(msg->height);
        const double completeness = expected_points_per_cloud_ > 0.0
            ? std::min(1.0, points / expected_points_per_cloud_)
            : 0.0;

        point_cloud_completeness_sum_ += completeness;
        ++point_cloud_count_;
    }

    double compute_sync_error_ms() const {
        if (sync_samples_ <= 0) {
            return -1.0;
        }
        return sync_sum_ms_ / static_cast<double>(sync_samples_);
    }

    double compute_frame_loss_rate() const {
        if (!has_first_image_stamp_ || !has_last_image_for_span_) {
            return 1.0;
        }
        const double span_sec = std::max(0.0, (last_image_stamp_for_span_ - first_image_stamp_).seconds());
        if (span_sec <= 0.0 || expected_image_fps_ <= 0.0) {
            return 1.0;
        }
        const double expected = span_sec * expected_image_fps_;
        if (expected <= 1e-6) {
            return 1.0;
        }
        const double ratio = static_cast<double>(image_count_) / expected;
        return std::clamp(1.0 - ratio, 0.0, 1.0);
    }

    double compute_blur_score() const {
        if (blur_samples_ <= 0 || min_blur_variance_ <= 0.0) {
            return 0.0;
        }
        const double blur_avg = blur_sum_ / static_cast<double>(blur_samples_);
        return std::clamp(blur_avg / min_blur_variance_, 0.0, 1.0) * 100.0;
    }

    double compute_point_cloud_completeness() const {
        if (point_cloud_count_ <= 0) {
            return 0.0;
        }
        return (point_cloud_completeness_sum_ / static_cast<double>(point_cloud_count_)) * 100.0;
    }

    std::string compute_level(double sync_ms, double frame_loss, double blur_score, double cloud_completeness, bool available) const {
        if (!available) {
            return "N/A";
        }

        if (sync_ms >= fail_sync_ms_ || frame_loss >= fail_frame_loss_rate_ ||
            blur_score <= fail_blur_score_ || cloud_completeness <= fail_cloud_completeness_) {
            return "FAIL";
        }

        if (sync_ms >= warn_sync_ms_ || frame_loss >= warn_frame_loss_rate_ ||
            blur_score <= warn_blur_score_ || cloud_completeness <= warn_cloud_completeness_) {
            return "WARN";
        }

        return "PASS";
    }

    void publish_quality() {
        weld_interface::msg::CollectionQuality msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg.header.stamp = this->now();
            msg.header.frame_id = "data_collect_quality_node";
            msg.session_dir = session_dir_;

            const double sync_ms = compute_sync_error_ms();
            const double frame_loss = compute_frame_loss_rate();
            const double blur_score = compute_blur_score();
            const double cloud_completeness = compute_point_cloud_completeness();

            const bool available = running_ && blur_samples_ > 0 && point_cloud_count_ > 0;
            msg.available = available;
            msg.sync_error_ms = sync_ms >= 0.0 ? static_cast<float>(sync_ms) : -1.0f;
            msg.frame_loss_rate = static_cast<float>(frame_loss);
            msg.blur_score = static_cast<float>(blur_score);
            msg.point_cloud_completeness = static_cast<float>(cloud_completeness);
            msg.level = compute_level(sync_ms, frame_loss, blur_score, cloud_completeness, available);
            msg.reason = available ? "ok" : reason_;
        }

        pub_quality_->publish(msg);

        acquisition_interfaces::msg::AcquisitionQuality generic_msg;
        generic_msg.header = msg.header;
        generic_msg.available = msg.available;
        generic_msg.session_dir = msg.session_dir;
        generic_msg.sync_error_ms = msg.sync_error_ms;
        generic_msg.frame_loss_rate = msg.frame_loss_rate;
        generic_msg.blur_score = msg.blur_score;
        generic_msg.point_cloud_completeness = msg.point_cloud_completeness;
        generic_msg.level = msg.level;
        generic_msg.reason = msg.reason;
        pub_acquisition_quality_->publish(generic_msg);
    }

    rclcpp::Subscription<weld_interface::msg::DataCollectStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<weld_interface::msg::CollectionQuality>::SharedPtr pub_quality_;
    rclcpp::Publisher<acquisition_interfaces::msg::AcquisitionQuality>::SharedPtr pub_acquisition_quality_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    bool running_{false};
    std::string session_dir_;
    std::string reason_{"waiting_collection"};

    double expected_image_fps_{15.0};
    double min_blur_variance_{120.0};
    double expected_points_per_cloud_{40000.0};
    double warn_sync_ms_{50.0};
    double fail_sync_ms_{100.0};
    double warn_frame_loss_rate_{0.05};
    double fail_frame_loss_rate_{0.10};
    double warn_blur_score_{70.0};
    double fail_blur_score_{40.0};
    double warn_cloud_completeness_{70.0};
    double fail_cloud_completeness_{40.0};

    int64_t image_count_{0};
    int64_t point_cloud_count_{0};
    int64_t blur_samples_{0};
    int64_t sync_samples_{0};
    double blur_sum_{0.0};
    double sync_sum_ms_{0.0};
    double point_cloud_completeness_sum_{0.0};

    rclcpp::Time first_image_stamp_;
    rclcpp::Time last_image_stamp_;
    rclcpp::Time last_image_stamp_for_span_;
    rclcpp::Time last_cloud_stamp_;
    bool has_first_image_stamp_{false};
    bool has_last_image_stamp_{false};
    bool has_last_image_for_span_{false};
    bool has_last_cloud_stamp_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataCollectQualityNode>());
    rclcpp::shutdown();
    return 0;
}
