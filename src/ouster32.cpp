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

#include <vector>

#define RESOLUTION 2048

size_t px_offset_2048[32] = {24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0, 24, 0};

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

    cv::Mat range_img(32, RESOLUTION, CV_8UC1, cv::Scalar(0));

    size_t H = 32;
    size_t W = RESOLUTION;


    std::cout<<cloud.size()<<std::endl;

    // std::vector<int> offset(H, -1);
    // for(int u = 0; u < H; ++u) {
    //     for(int v =0; v < W; ++v) {
    //         int index = u*W + v;
    //         const auto & pt = cloud[index];
    //         if(pt.range != 0) {
    //             float angle = atan2(pt.y, pt.x)*180.0/M_PI;
    //             float ang_int_f = (angle)/(360.0 / 2048.0);
    //             int ang_int = 0;
    //             if(ang_int_f < 0) {
    //                 ang_int = int(ang_int_f) - 1;
    //             } else {
    //                 ang_int = int(ang_int_f);
    //             }
    //             ang_int = 1041 - ang_int;

    //             // std::cout<<"Row: "<<u<<" Column: "<<v<<std::endl;
    //             // std::cout<<"Angle: "<<angle<< " Angle to integer: "<<ang_int<<std::endl;
    //             // std::cout<<"Previous: " <<offset[u]<<" Current: "<<ang_int - v + 2<< " Given: "<<int(px_offset_2048[u])<<std::endl;
    //             // std::cout<<"-------------------------------\n"<<std::endl;

    //             if(offset[u]!= -1) {
    //                 if(offset[u]!= (ang_int - v + 2)%W) {
    //                     // std::cout<<"[Shit] Previous: " <<offset[u]<<" Current: "<<ang_int - v + 2<< " Given: "<<int(px_offset_2048[u])<<std::endl;
    //                     // std::cout<<"+++++++++++++++++\n"<<std::endl;
    //                     offset[u] = (ang_int - v + 2)%W;
    //                 }
    //             }
    //             offset[u] = (ang_int - v + 2)%W;
    //         }
    //     }
    // }



    for (size_t u = 0; u < H; u++) {
        for (size_t v = 0; v < W; v++) {
            size_t vv;
            vv = (v + W - px_offset_2048[u]) % W;
            const size_t index = u * W + vv;
            const auto& pt = cloud[index];

            if (pt.range == 0) {
                range_img.at<uint8_t>(u, v) = uint8_t(0);
            } else {
                range_img.at<uint8_t>(u, v) = 245 - std::min(std::round(pt.range * 1/200.0), static_cast<double>(245));
            }
        }
    }


    cv::imshow("Shit", range_img);
    cv::waitKey(1);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_starboard/os_cloud_node/points", 1, OnSubscribeLiDARPointCloud);

    ros::spin();

    return 0;
}
