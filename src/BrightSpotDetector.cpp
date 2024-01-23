#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

class BrightSpotDetectorNode : public rclcpp::Node
{
public:
	BrightSpotDetectorNode();

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	cv::Mat processImage(const cv::Mat& img);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
};

BrightSpotDetectorNode::BrightSpotDetectorNode() : Node("bright_spot_detector_node")
{
	this->declare_parameter("brightness_threshold", 250);
	this->declare_parameter("area_threshold", 25);

	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/infra1/image_rect_raw", qos, std::bind(&BrightSpotDetectorNode::image_callback, this, std::placeholders::_1));

	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
		"/bright_spot_detector", qos);
}

void BrightSpotDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat processed_image = processImage(cv_ptr->image);

		// Convert the processed OpenCV Image back to ROS Image message
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = processed_image;

		// Publish the processed image
		_image_pub->publish(*out_msg.toImageMsg().get());

	} catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
}

cv::Mat BrightSpotDetectorNode::processImage(const cv::Mat& img)
{
	// Get parameter values
	int brightness_threshold, area_threshold;
	this->get_parameter("brightness_threshold", brightness_threshold);
	this->get_parameter("area_threshold", area_threshold);

	// Threshold the image to isolate bright areas
	cv::Mat img_thresholded;
	cv::threshold(img, img_thresholded, brightness_threshold, 255, cv::THRESH_BINARY);

	// Find contours
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(img_thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Convert the grayscale image to BGR for displaying colored circles
	cv::Mat color_image;
	cv::cvtColor(img, color_image, cv::COLOR_GRAY2BGR);

	// Keep track of biggest contour
	int biggest_radius = 0;
	cv::Point biggest_contour_center;

	// Process each contour
	for (const auto& contour : contours) {
		double area = cv::contourArea(contour);
		if (area > area_threshold) {
			cv::Moments moment = cv::moments(contour);
			cv::Point center(static_cast<int>(moment.m10 / moment.m00), static_cast<int>(moment.m01 / moment.m00));

			// Calculate radius proportional to the square root of the area
			int radius = static_cast<int>(sqrt(area) / 2);
			cv::circle(color_image, center, radius, cv::Scalar(0, 0, 255), 1, cv::LINE_AA); // Red circle

			if (radius > biggest_radius) {
				biggest_radius = radius;
				biggest_contour_center = center;
			}
		}
	}

	// Circle biggest contour in green
	cv::circle(color_image, biggest_contour_center, biggest_radius, cv::Scalar(0, 255, 0), 1, cv::LINE_AA); // Green circle

	return color_image; // Return the color image with circles marking the blobs' centers
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BrightSpotDetectorNode>());
	rclcpp::shutdown();
	return 0;
}
