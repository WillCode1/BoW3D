#include <string>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "LinK3D_Extractor.h"
#include "BoW3D.h"

using namespace std;
using namespace BoW3D;

// Parameters of LinK3D
int nScans = 32; // Number of LiDAR scan lines
float scanPeriod = 0.1;
float minimumRange = 0.1;
float distanceTh = 0.4;
int matchTh = 6;

// Parameters of BoW3D
float thr = 3.5;
int thf = 5;
int num_add_retrieve_features = 5;

std::shared_ptr<BoW3D::LinK3D_Extractor> pLinK3dExtractor = std::make_shared<BoW3D::LinK3D_Extractor>(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
std::shared_ptr<BoW3D::BoW3D> pBoW3D = std::make_shared<BoW3D::BoW3D>(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

// #define VEL_TIMESTAMP_TYPE float
#define VEL_TIMESTAMP_TYPE double
// #define VEL_TIMESTAMP_FIELD time
#define VEL_TIMESTAMP_FIELD timestamp

namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    VEL_TIMESTAMP_TYPE VEL_TIMESTAMP_FIELD;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (VEL_TIMESTAMP_TYPE, VEL_TIMESTAMP_FIELD, VEL_TIMESTAMP_FIELD)
    (std::uint16_t, ring, ring)
)

void pcl_convert(pcl::PointCloud<velodyne_ros::Point> &pl_orig_velo, pcl::PointCloud<PointType> &current_cloud)
{
    int plsize = pl_orig_velo.points.size();
    for (int i = 0; i < plsize; i++)
    {
        PointType added_pt;
        added_pt.x = pl_orig_velo.points[i].x;
        added_pt.y = pl_orig_velo.points[i].y;
        added_pt.z = pl_orig_velo.points[i].z;
        added_pt.intensity = pl_orig_velo.points[i].intensity;
        current_cloud.points.push_back(added_pt);
    }
}

void read_rosbag(const std::string &bagfile, const std::vector<std::string> &topics,
                 std::shared_ptr<BoW3D::LinK3D_Extractor> &linK3dExtractor,
                 std::shared_ptr<BoW3D::BoW3D> &BoW3D)
{
    rosbag::Bag bag;

    try
    {
        bag.open(bagfile, rosbag::bagmode::Read);
    }
    catch(const std::exception& e)
    {
        std::cout << bagfile << '\n';
        std::cerr << e.what() << '\n';
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ROS_INFO("Test rosbag %s...", bagfile.c_str());

    int cnt = 0;
    int load_keyframe = 0;

    for (const rosbag::MessageInstance& msg : view)
    {
        if (!msg.isType<sensor_msgs::PointCloud2>())
            continue;

        cnt++;

        if (cnt % 10 != 0)
            continue;

        load_keyframe++;

        sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
        pcl::PointCloud<velodyne_ros::Point> pl_orig_velo;
        pcl::PointCloud<PointType>::Ptr current_cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*cloud, pl_orig_velo);
        pcl_convert(pl_orig_velo, *current_cloud);
        std::shared_ptr<Frame> pCurrentFrame = std::make_shared<Frame>(linK3dExtractor, current_cloud);
        BoW3D->update(pCurrentFrame);
    }
    ROS_INFO("Read lidar keyframe %d.", load_keyframe);

    bag.close();
}

void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<velodyne_ros::Point> pl_orig_velo;
    pcl::PointCloud<PointType>::Ptr current_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, pl_orig_velo);
    pcl_convert(pl_orig_velo, *current_cloud);
    std::shared_ptr<Frame> pCurrentFrame = std::make_shared<Frame>(pLinK3dExtractor, current_cloud);

    int loopFrameId = -1;
    Eigen::Matrix3d loopRelR;
    Eigen::Vector3d loopRelt;
    pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);
    if (loopFrameId != -1)
        ROS_WARN("%d", loopFrameId);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BoW3D");
    ros::NodeHandle nh;

    string bag_path;
    ros::param::param("bag_path", bag_path, std::string("/home/will/data/work_data/qingdao3.bag"));
    std::vector<std::string> topics = {"/drivers/top_lidar_origin"};

    read_rosbag(bag_path, topics, pLinK3dExtractor, pBoW3D);

    ros::Subscriber sub_pcl = nh.subscribe(topics[0], 200000, pcl_callback);
    // test_rosbag(bag_path, topics, pLinK3dExtractor, pBoW3D);

    ros::spin();

    return 0;
}
