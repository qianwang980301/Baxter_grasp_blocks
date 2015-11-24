// pcl_grabing.h header file
// Zhiang Chen, Nov 2015
// for pcl in final project 

#ifndef PCL_GRABING_H_
#define PCL_GRABING_H_

#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#define roughHeight 0.5
#define HeightRange 0.05
#define roughColor_R 10
#define roughColor_G 10
#define roughColor_B 10
#define ColorRange 0.1
#define TableRadius 0.2
#define HandMinHeight 0.1
#define BlockMaxHeight 0.1
#define BlockTopRadius 0.005
#define BlockRadius 0.05

class Pcl_grabing: public CwruPclUtils
{
public:
    Pcl_grabing(ros::NodeHandle* nodehandle);  // constructor
    bool findTableTop(); //
    bool checkForHand();
    bool isBlock(); //
    geometry_msgs::Pose getBlockPose(); //
    Eigen::Vector3d getColor(); //
    //float getWidth();
    //float getHeight();
private:
    void update_kinect_points();
    void transform_clr_kinect_cloud(Eigen::Affine3f A);
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pclKinect_clr_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr display_ptr_;
    ros::Publisher table_publisher;  
    geometry_msgs::Pose BlockPose;
    Eigen::Vector3d BlockColor;
    double TableHeight;
    Eigen::Vector3d TableColor;
    Eigen::Vector3f TableCentroid;
    
    Eigen::Vector3f Block_Major;
    Eigen::Vector3f Block_Normal;
    Eigen::Vector3f BlockTopCentroid;
};

#endif // PCL_GRABING_H_
