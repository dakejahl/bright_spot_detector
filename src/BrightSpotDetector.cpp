#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
	void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
	cv::Mat overlayPointCloud(const cv::Mat& rgb_image, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _latest_cloud;
	std::mutex _cloud_mutex;
};

PointCloudOverlayNode::PointCloudOverlayNode() : Node("point_cloud_overlay_node"), _latest_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/color/image_raw", qos, std::bind(&PointCloudOverlayNode::image_callback, this, std::placeholders::_1));

	_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"/visual_slam/vis/observations_cloud", qos, std::bind(&PointCloudOverlayNode::point_cloud_callback, this, std::placeholders::_1));

	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
		"/overlay_image", qos);
}

void PointCloudOverlayNode::image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
	cv::Mat rgb_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	{
		std::lock_guard<std::mutex> lock(_cloud_mutex);
		cloud = _latest_cloud;
	}

	if (!rgb_image.empty()) {
		// TODO: timestamp pointcloud
		if (cloud) {
			cv::Mat overlayed_image = overlayPointCloud(rgb_image, cloud);
			// Convert to ROS message and publish
			sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(img_msg->header, "bgr8", overlayed_image).toImageMsg();
			_image_pub->publish(*msg);
		} else {
			_image_pub->publish(*img_msg);
		}

	}
}

void PointCloudOverlayNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
	std::lock_guard<std::mutex> lock(_cloud_mutex);
	pcl::fromROSMsg(*cloud_msg, *_latest_cloud);
}

cv::Point2d convert3DPointTo2D(const pcl::PointXYZ& point) {

	Intrinsic of "Infrared" / 640x480 / {RGB8/BGR8/RGBA8/BGRA8/UYVY}
	Width:        640
	Height:       480
	PPX:          325.685089111328
	PPY:          238.197769165039
	Fx:           387.751678466797
	Fy:           387.751678466797
	Distortion:   Brown Conrady
	Coeffs:       0       0       0       0       0
	FOV (deg):    79.06 x 63.51

	double fx = 387.7516784667969;
	double cx = 325.6850891113281;
	double fy = 387.7516784667969;
	double cy = 238.19776916503906;


    // Camera intrinsic parameters
    double fx = 242.344802856445;
    double cx = 243.553192138672;

    double fy = 242.344802856445;
    double cy = 133.873596191406;

    // Assuming point.z is not zero
    double x = point.x / point.z;
    double y = point.y / point.z;

    // Applying the intrinsic parameters
    double u = fx * x + cx;
    double v = fy * y + cy;

    return cv::Point2d(u, v);
}

cv::Mat PointCloudOverlayNode::overlayPointCloud(const cv::Mat& rgb_image, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cv::Mat overlayed_image = rgb_image.clone();

    for (const auto& point : cloud->points) {
        if (point.z != 0) {  // Ensure that point.z is not zero to avoid division by zero
            cv::Point2d image_point = convert3DPointTo2D(point);
            // Check if the point falls within the image frame
            if (image_point.x >= 0 && image_point.x < overlayed_image.cols &&
                image_point.y >= 0 && image_point.y < overlayed_image.rows) {
                cv::circle(overlayed_image, image_point, 3, cv::Scalar(0, 0, 255), -1); // Draw red circle
            }
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
