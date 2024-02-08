#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudOverlayNode : public rclcpp::Node
{
public:
	PointCloudOverlayNode();

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg);
	void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
	cv::Mat overlayPointCloud(const cv::Mat& ir_image, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	cv::Point2d convert3DPointTo2D(const pcl::PointXYZ& point);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _latest_cloud;
	std::mutex _cloud_mutex;
	std::mutex _camera_info_mutex;

	// Camera intrinsic parameters
	double fx, fy, cx, cy;
	bool _intrinsic_set = false;
};

PointCloudOverlayNode::PointCloudOverlayNode() : Node("point_cloud_overlay_node"), _latest_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/infra1/image_rect_raw", qos, std::bind(&PointCloudOverlayNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/camera/infra1/camera_info", qos, std::bind(&PointCloudOverlayNode::camera_info_callback, this, std::placeholders::_1));

	_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"/visual_slam/vis/observations_cloud", qos, std::bind(&PointCloudOverlayNode::point_cloud_callback, this, std::placeholders::_1));

	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
		"/overlay_image", qos);
}

void PointCloudOverlayNode::image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
	cv::Mat ir_image = cv_bridge::toCvCopy(img_msg, "mono8")->image;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	{
		std::lock_guard<std::mutex> lock(_cloud_mutex);
		cloud = _latest_cloud;
	}

	if (!ir_image.empty() && _intrinsic_set) {
		if (cloud) {
			cv::Mat overlayed_image = overlayPointCloud(ir_image, cloud);
			cv_bridge::CvImage out_msg;
			out_msg.header = img_msg->header;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = overlayed_image;
			_image_pub->publish(*out_msg.toImageMsg().get());

		} else {
			_image_pub->publish(*img_msg);
		}
	}
}

void PointCloudOverlayNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info_msg)
{
	std::lock_guard<std::mutex> lock(_camera_info_mutex);
	fx = info_msg->k[0]; // Focal length in x
	fy = info_msg->k[4]; // Focal length in y
	cx = info_msg->k[2]; // Principal point x
	cy = info_msg->k[5]; // Principal point y
	_intrinsic_set = true;
}

void PointCloudOverlayNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
	std::lock_guard<std::mutex> lock(_cloud_mutex);
	pcl::fromROSMsg(*cloud_msg, *_latest_cloud);
}

cv::Point2d PointCloudOverlayNode::convert3DPointTo2D(const pcl::PointXYZ& point)
{
	if (!_intrinsic_set)
		return cv::Point2d(-1, -1); // Intrinsic parameters not set

	// Assuming point.z is not zero
	double x = point.x / point.z;
	double y = point.y / point.z;

	// Applying the intrinsic parameters
	std::lock_guard<std::mutex> lock(_camera_info_mutex);
	double u = fx * x + cx;
	double v = fy * y + cy;

	return cv::Point2d(u, v);
}

cv::Mat PointCloudOverlayNode::overlayPointCloud(const cv::Mat& ir_image, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Convert IR image to BGR for displaying colored circles
	cv::Mat overlayed_image;
	cv::cvtColor(ir_image, overlayed_image, cv::COLOR_GRAY2BGR);

	for (const auto& point : cloud->points) {
		if (point.z != 0) {  // Ensure that point.z is not zero to avoid division by zero
			cv::Point2d image_point = convert3DPointTo2D(point);

			// Check if the point falls within the image frame
			if (image_point.x >= 0 && image_point.x < overlayed_image.cols &&
				image_point.y >= 0 && image_point.y < overlayed_image.rows) {
				cv::circle(overlayed_image, image_point, 3, cv::Scalar(0, 0, 255), -1); // Draw red circle
			} else {
				RCLCPP_INFO(get_logger(), "point outside frame!");
			}

		} else {
			RCLCPP_INFO(get_logger(), "point.z is zero!");
		}
	}

	return overlayed_image;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PointCloudOverlayNode>());
	rclcpp::shutdown();
	return 0;
}
