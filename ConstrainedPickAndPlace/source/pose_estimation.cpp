#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/random.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace pcl::common;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;




/***Forward declarations***/
/***ANBU***/
Matrix4f ICP(PointCloud<PointNormal>::Ptr scene, PointCloud<PointNormal>::Ptr object, PointCloud<PointNormal>::Ptr cloud_out, int iterations);
Matrix4f Global_alignment(PointCloud<PointNormal>::Ptr object_global, PointCloud<PointNormal>::Ptr scene,
                      PointCloud<PointNormal>::Ptr cloud_out, int iterations);
void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);

/***Preprocessing***/
void downsample(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, double leafsize);
void passthrough(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, double cut_off);
void remove_planes(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, int thresh_inliers);





/*
 * downsample
 * remove table plane
 * passthrough
 * (moving average filter) filter:=temporal
 *
*/
ros::Publisher pub;
PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
PointCloud<PointT>::Ptr cad_model(new PointCloud<PointT>);

double cut_off_dist = 0.6;
bool visualization_on = true;
bool pose_estimate = false;
void cb_get_pointcloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if(pose_estimate)
    {
        PointCloud<PointT>::Ptr cloud_filtered(new PointCloud<PointT>);

        pcl::fromROSMsg(*cloud_msg, *cloud_filtered);
        cout << "captured a cloud with #points: " << endl;
        cout << cloud_filtered->points.size() << endl;

        passthrough(cloud_filtered, cloud_filtered, cut_off_dist);
        downsample(cloud_filtered, cloud_filtered, 0.005); //0.005
        remove_planes(cloud_filtered, cloud_filtered, 1000);

//        pcl::io::savePCDFile("/home/Desktop/cloud_filtered.pcd", *cloud_filtered); //http://pointclouds.org/documentation/tutorials/writing_pcd.php
//        std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;

        cout << "After preprocessing -- #points: " << cloud_filtered->points.size() << endl;

        PointCloud<PointNormal>::Ptr cad_copy(new PointCloud<PointNormal>);
        PointCloud<PointNormal>::Ptr cad_copy2(new PointCloud<PointNormal>);
        *cad_copy = *cad_model;
        *cad_copy2 = *cad_model;

        PointCloud<PointNormal>::Ptr global_aligned(new PointCloud<PointNormal>);
        PointCloud<PointNormal>::Ptr local_aligned(new PointCloud<PointNormal>);

        Matrix4f T_global  = Global_alignment(cad_model, cloud_filtered, global_aligned, 5000);
        Matrix4f T_local     = ICP(cloud_filtered, global_aligned, local_aligned,50);

        cout << "Got the final pose estimate: T_global * T_local" << endl;
        Matrix4f pose = T_local * T_global;
        cout << pose << endl;

        /***Verification***/
//        PointCloud<PointXYZ>::Ptr original_cloud(new PointCloud<PointXYZ>::Ptr);
//        pcl::fromROSMsg(*cloud_msg, original_cloud);
        transformPointCloud(*cad_copy, *cad_copy, T_global);
        if(visualization_on)
        {
            PCLVisualizer v("Only global transform applied");
            v.addPointCloud<PointNormal>(cloud_filtered, PointCloudColorHandlerCustom<PointNormal>(cloud_filtered, 0, 255, 0), "sceneyo");
            v.addPointCloud<PointNormal>(cad_copy, PointCloudColorHandlerCustom<PointNormal>(cad_copy, 255, 0, 0),"cadcopy");
//            v.addCoordinateSystem(0.5, "cadcopy");
//            v.addCoordinateSystem(1.0, "sceneyo");
            v.spin();
        }
        transformPointCloud(*cad_copy2, *cad_copy2, pose);
        if(visualization_on)
        {
            PCLVisualizer v("Both global and local transforms applied");
            v.addPointCloud<PointNormal>(cloud_filtered, PointCloudColorHandlerCustom<PointNormal>(cloud_filtered, 0, 255, 0), "sceneyoyo");
            v.addPointCloud<PointNormal>(cad_copy2, PointCloudColorHandlerCustom<PointNormal>(cad_copy2, 0, 0, 255),"cadcopy2");
//            v.addCoordinateSystem(0.5, "cadcopy2");
//            v.addCoordinateSystem(1.0, "sceneyoyo");
            v.spin();
        }
        pose_estimate = false;
        /***Publish the pose estimate***/
        auto pose_data_ptr = pose.data();
        vector<float> pose_array;
        for(int i = 0; i < 15; i++ )
            pose_array.push_back(pose_data_ptr[i]);

        std_msgs::Float32MultiArray msg;
        msg.data = pose_array;
        pub.publish(msg);
    }
}

void cb_give_transform(const std_msgs::Bool msg)
{
    cout << "In callback make transform: \n";
    cout << "Message bool: " << msg.data << endl;
    pose_estimate = msg.data;
    cout << "make_transform: " << pose_estimate << endl;
}



int main(int argc, char** argv)
{
    //loadPCDFile("/home/soren/8-semester/8_semester/projekt/catkin_ws/devel/lib/ur_caros_example/picasso_cad.pcd",*cad_model);

    if(argc < 3)
    {
        cout << "Usage: ./pose_estimation [1/0] [cut_off_dist]" << endl;
        cout << "1 for Visulization ON" << endl;
        cout << "0 for visualization OFF" << endl;
        return -1;
    }

    if(argv[1][0] == '0')
        visualization_on = false;
    else
        visualization_on = true;

    cut_off_dist = stod(argv[2]);
    cout << "argv[1]: " << argv[1] << endl;
    cout << "Visualization status: " << visualization_on << endl;
    cout << "Cutoff distance: " << cut_off_dist << endl;


    auto packagePath = ros::package::getPath("ur_caros_example");
    loadPCDFile(packagePath + "/pointclouds/snemand_cad.pcd",*cad_model);

    cout << "Loaded cad model with #points: " << cad_model->points.size() << endl;
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;
    ros::Subscriber sub     = nh.subscribe("/camera/depth_registered/points", 1, cb_get_pointcloud);
    ros::Subscriber sub2    = nh.subscribe("give_transform", 1, cb_give_transform);
    pub = nh.advertise<std_msgs::Float32MultiArray>("transform",10);

    //ros::Duration(5).sleep();



    ros::spin();



    return 0;
}

/***Preprocessing***/
void downsample(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, double leafsize)
{
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(leafsize, leafsize, leafsize);
    sor.filter(*cloud_out);
}
void passthrough(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, double cut_off)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,cut_off);
    pass.filter(*cloud_out);
}

void remove_planes(PointCloud<PointT>::Ptr cloud_in, PointCloud<PointT>::Ptr cloud_out, int thresh_inliers)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
for(int i = 0; i < 5; i++)
{
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                          << coefficients->values[1] << " "
//                                          << coefficients->values[2] << " "
//                                          << coefficients->values[3] << std::endl;
//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//      for (size_t i = 0; i < inliers->indices.size (); ++i)
//        std::cerr << inliers->indices[i] << "    " << cloud_in->points[inliers->indices[i]].x << " "
//                                                   << cloud_in->points[inliers->indices[i]].y << " "
//                                                   << cloud_in->points[inliers->indices[i]].z << std::endl;


  pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
  for(int i = 0; i < inliers->indices.size(); i++)
  {
      plane->points.push_back(cloud_in->points[inliers->indices[i]]);
  }
  if(plane->points.size() > thresh_inliers)
  {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_in);
  }
  break;
}

  *cloud_out = *cloud_in;
}

/***ANBU***/
Matrix4f ICP(PointCloud<PointNormal>::Ptr scene, PointCloud<PointNormal>::Ptr object, PointCloud<PointNormal>::Ptr cloud_out, int iterations)
{
// Show
    if(visualization_on)
    {
    PCLVisualizer v("Before local alignment");
    v.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
    v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
//    v.addCoordinateSystem(0.5, "object");
//    v.addCoordinateSystem(1.0, "scene");

    v.spin();
    }


    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);

    // Set ICP parameters
    const size_t iter = iterations;
    const float thressq = 0.01 * 0.01;

    // Start ICP
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
    {
        ScopeTime t("ICP");
        cout << "Starting ICP..." << endl;
        for(size_t i = 0; i < iter; ++i) {
            // 1) Find closest points
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

            // Threshold and create indices for object/scene and compute RMSE
            vector<int> idxobj;
            vector<int> idxscn;
            for(size_t j = 0; j < idx.size(); ++j) {
                if(distsq[j][0] <= thressq) {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }

            // 2) Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);

            // 3) Apply pose
            transformPointCloud(*object_aligned, *object_aligned, T);

            // 4) Update result
            pose = T * pose;
        }

        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);

        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing

    // Show result
    if(visualization_on)
    {
        PCLVisualizer v("After local alignment");
        v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
//        v.addCoordinateSystem(0.5, "object_aligned");
//        v.addCoordinateSystem(1.0, "scene");
        v.spin();
    }
    *cloud_out = *object_aligned;
	return pose;
}

Matrix4f Global_alignment(PointCloud<PointNormal>::Ptr object, PointCloud<PointNormal>::Ptr scene, PointCloud<PointNormal>::Ptr cloud_out, int iterations)
{
    // Show
    if(visualization_on)
       {
           PCLVisualizer v("Before global alignment");
           v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
           v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
    //           v.addCoordinateSystem(0.5, "object");
    //           v.addCoordinateSystem(1.0, "scene");
           v.spin();
       }

       // Compute surface normals
       {
           ScopeTime t("Surface normals");
           NormalEstimation<PointT,PointT> ne;
           ne.setKSearch(10);

           ne.setInputCloud(object);
           ne.compute(*object);

           ne.setInputCloud(scene);
           ne.compute(*scene);
       }

       // Compute shape features
       PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
       PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
       {
           ScopeTime t("Shape features");

           SpinImageEstimation<PointT,PointT,FeatureT> spin;
           spin.setRadiusSearch(0.05);

           spin.setInputCloud(object);
           spin.setInputNormals(object);
           spin.compute(*object_features);

           spin.setInputCloud(scene);
           spin.setInputNormals(scene);
           spin.compute(*scene_features);
       }

       // Find feature matches
       Correspondences corr(object_features->size());
       {
           ScopeTime t("Feature matches");
           for(size_t i = 0; i < object_features->size(); ++i) {
               corr[i].index_query = i;
               nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
           }
       }

       // Show matches
       if(visualization_on)
       {
           PCLVisualizer v("Matches");
           v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
           v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
           v.addCorrespondences<PointT>(object, scene, corr, 1);
           v.spin();
       }

       // Create a k-d tree for scene
       search::KdTree<PointNormal> tree;
       tree.setInputCloud(scene);

       // Set RANSAC parameters
       const size_t iter = iterations;
       const float thressq = 0.01 * 0.01;

       // Start RANSAC
       Matrix4f pose = Matrix4f::Identity();
       PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
       float penalty = FLT_MAX;
       {
           ScopeTime t("RANSAC");
           cout << "Starting RANSAC..." << endl;
           UniformGenerator<int> gen(0, corr.size() - 1);
           for(size_t i = 0; i < iter; ++i) {
               if((i + 1) % 100 == 0)
                   cout << "\t" << i+1 << endl;
               // Sample 3 random correspondences
               vector<int> idxobj(3);
               vector<int> idxscn(3);
               for(int j = 0; j < 3; ++j) {
                   const int idx = gen.run();
                   idxobj[j] = corr[idx].index_query;
                   idxscn[j] = corr[idx].index_match;
               }

               // Estimate transformation
               Matrix4f T;
               TransformationEstimationSVD<PointNormal,PointNormal> est;
               est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);

               // Apply pose
               transformPointCloud(*object, *object_aligned, T);

               // Validate
               vector<vector<int> > idx;
               vector<vector<float> > distsq;
               tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

               // Compute inliers and RMSE
               size_t inliers = 0;
               float rmse = 0;
               for(size_t j = 0; j < distsq.size(); ++j)
                   if(distsq[j][0] <= thressq)
                       ++inliers, rmse += distsq[j][0];
               rmse = sqrtf(rmse / inliers);

               // Evaluate a penalty function
               const float outlier_rate = 1.0f - float(inliers) / object->size();
               //const float penaltyi = rmse;
               const float penaltyi = outlier_rate;

               // Update result
               if(penaltyi < penalty) {
                   cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                   penalty = penaltyi;
                   pose = T;
               }
           }

           transformPointCloud(*object, *object_aligned, pose);

           // Compute inliers and RMSE
           vector<vector<int> > idx;
           vector<vector<float> > distsq;
           tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
           size_t inliers = 0;
           float rmse = 0;
           for(size_t i = 0; i < distsq.size(); ++i)
               if(distsq[i][0] <= thressq)
                   ++inliers, rmse += distsq[i][0];
           rmse = sqrtf(rmse / inliers);

           // Print pose
           cout << "Got the following pose:" << endl << pose << endl;
           cout << "Inliers: " << inliers << "/" << object->size() << endl;
           cout << "RMSE: " << rmse << endl;
       } // End timing

       // Show result
       if(visualization_on)
       {
           PCLVisualizer v("After global alignment");
           v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
           v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
//           v.addCoordinateSystem(0.5, "object_aligned");
//           v.addCoordinateSystem(1.0, "scene");
           v.spin();
       }

       *cloud_out = *object_aligned;
	return pose;

}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}
