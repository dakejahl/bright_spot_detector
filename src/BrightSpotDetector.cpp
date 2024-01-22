#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat processImage(const cv::Mat& img) {
    // Apply Gaussian blur
    cv::Mat img_processed;
    int blurRadius = 5; // Set Gaussian blur radius. Must be an odd number to ensure a central pixel for symmetric blurring.
    if (blurRadius % 2 == 0) ++blurRadius;
    cv::GaussianBlur(img, img_processed, cv::Size(blurRadius, blurRadius), 0);

    // Find the brightest spot
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(img_processed, &minVal, &maxVal, &minLoc, &maxLoc);

    // Mark the brightest spot on the original image
    cv::Mat result = img.clone();
    cv::circle(result, maxLoc, blurRadius, cv::Scalar(255, 0, 0), 2);

    return result;
}

class BrightSpotDetectorNode : public rclcpp::Node {
public:
    BrightSpotDetectorNode() : Node("bright_spot_detector_node") {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/infra1/image_rect_raw", qos, std::bind(&BrightSpotDetectorNode::image_callback, this, std::placeholders::_1));

        _image_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "/processed_image", qos);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            cv::Mat processed_image = processImage(cv_ptr->image);

            // Convert the processed OpenCV Image back to ROS Image message
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header; // Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever encoding is suitable
            out_msg.image = processed_image;

            // Publish the processed image
            _image_pub->publish(*out_msg.toImageMsg().get());

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightSpotDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
