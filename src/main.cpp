#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>




#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#define RESOLUTION 2048

size_t px_offset_1024[64] = {18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 18, 12, 6, 0, 19, 12, 6, 0, 19, 13, 6, 0};
size_t px_offset_2048[64] = {37, 24, 12, 0, 37, 24, 12, 0, 37, 24, 12, 0, 37, 25, 12, 1, 36, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 1, 37, 25, 13, 2, 38, 26, 14, 2, 38, 26, 14, 2, 38, 26, 14, 1, 38, 26, 14, 1};

struct EIGEN_ALIGN16 OusterPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

using Cloud = pcl::PointCloud<OusterPoint>;

void OnSubscribeLiDARPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<OusterPoint> cloud{};
    pcl::fromROSMsg(*msg, cloud);

    cv::Mat range_img(64, RESOLUTION, CV_8UC1, cv::Scalar(0));

    size_t H = 64;
    size_t W = RESOLUTION;

    for (size_t u = 0; u < H; u++) {
        for (size_t v = 0; v < W; v++) {
            size_t vv;
            if(RESOLUTION == 1024) {
                vv = (v + W - px_offset_1024[u]) % W;
            } else if(RESOLUTION == 2048) {
                vv = (v + W - px_offset_2048[u]) % W;
            }
            const size_t index = u * W + vv;
            const auto& pt = cloud[index];

            if (pt.range == 0) {
                range_img.at<uint8_t>(u, v) = uint8_t(0);
            } else {
                range_img.at<uint8_t>(u, v) = 255 - std::min(std::round(pt.range * 1/200.0), static_cast<double>(255));
            }
        }
    }


    cv::imshow("Shit", range_img);
    cv::waitKey(1);


}

void OnSubscribeLiDARRangeImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img = cv_ptr->image.clone();

    std::cout<<"Image size: "<<img.rows <<" by "<<img.cols<<std::endl;

    cv::imshow("Given Image", img);
    cv::waitKey(1);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_front/os_cloud_node/points", 1, OnSubscribeLiDARPointCloud);
    ros::Subscriber range_img_sub = nh.subscribe<sensor_msgs::Image>("/lidar_front/img_node/range_image", 1, OnSubscribeLiDARRangeImage);

    ros::spin();

    return 0;
}
