/********************************************************
 * Use ransac to measure wood brick of arbitery shape
 * *****************************************************/

// Working version
#include <iostream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

// This section is for normal algorithm
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sys/time.h>
#include <ctime>

// This section is for euclidean clustering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/* Returns the amount of milliseconds elapsed since the UNIX epoch. */
uint64_t time_ms()
{
 struct timeval tv;
 gettimeofday(&tv, NULL);
 uint64_t ret = tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000;
 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 ret += (tv.tv_sec * 1000);
 return ret;
}

uint64_t time_us()
{
 struct timeval tv;
 gettimeofday(&tv, NULL);
 uint64_t ret = tv.tv_usec;
 /* Adds the seconds (10^0) after converting them to microseconds (10^-6) */
 ret += (tv.tv_sec * 1000 * 1000);
 return ret;
}

int main(int argv, char **args) {
    /** Timing code */
    uint64_t tstart_ms, tend_ms, tstart_us, tend_us;
    tstart_ms = time_ms();
    tstart_us = time_us();
    
    /** Read the point cloud from documnents */
    PointCloudT::Ptr cloud_in (new PointCloudT);
    PointCloudT::Ptr cloud_in_visual (new PointCloudT);
    if (pcl::io::loadPCDFile("woodonly.pcd",*cloud_in)<0) {
        PCL_ERROR ("Error loading cloud 4by4_3faces_pointcloud.ply");
    }
    cloud_in_visual  = cloud_in;

/* yuhan
    pcl::StatisticalOutlierRemoval<PointT> presor(true);
    presor.setInputCloud(cloud_in);
    presor.setMeanK(8);
    presor.setStddevMulThresh(0.5);
    presor.filter(*cloud_in_visual);

    cloud_in = cloud_in_visual;
*/

    /** Parse commands */
    bool showNormals = false, showPlanes = false;
    std::string woodarg = "";
    
    if (argv < 2) {
        std::cout << "Usage: " << args[0] << " " << "wood_num [-np]\n";
        exit(1);
    }
    woodarg = args[1];
    // Parse any remaining input arguments
    for (int iarg = 2; iarg < argv; iarg++) {
        std::string arg = args[iarg];
        // Option
        if (arg[0] = '-') {
            for (int istr = 1; istr < arg.length(); istr++) {
                if (arg[istr] == 'n') {
                    showNormals = true;
                } else if (arg[istr] == 'p') {
                    showPlanes = true;
                }
            }
        }
    }

    /** Select clicked points based on command */
    PointT firstClick(0, 0, 0), secondClick(0, 0, 0);
    if (woodarg == "1") {
        // wood1 (V not bad after change click, shrunken dims)
        firstClick.x = 0.9547; firstClick.y = 0.622662; firstClick.z = 0.9887;
        secondClick.x = 1.0003; secondClick.y = 0.621751; secondClick.z = 1.05196;
    } else if (woodarg == "2") {
        // wood2 (V not bad after change click)
        firstClick.x = 0.950628; firstClick.y = 0.445224; firstClick.z = 0.0538721;
        secondClick.x = 0.988609; secondClick.y = 0.457071; secondClick.z = 0.102262;
    } else if (woodarg == "3a") {
        // wood3 short (V good except shrunken dims)
        firstClick.x = 0.967892; firstClick.y = 0.534155; firstClick.z = 0.974875;
        secondClick.x = 0.978867; secondClick.y = 0.526969; secondClick.z = 1.04312;
    } else if (woodarg == "3b") {
        // wood3 long (V good)
        firstClick.x = 1.11743; firstClick.y = -0.0891154; firstClick.z = 1.04574;
        secondClick.x = 1.14564; secondClick.y = -0.12534; secondClick.z = 0.9848;
    } else if (woodarg == "4a") {
        // wood4 front (V not bad after change click)
        firstClick.x = 0.911129; firstClick.y = -0.0618588; firstClick.z = 1.05675;
        secondClick.x = 0.992603; secondClick.y = 0.0514742; secondClick.z = 1.02301;
    } else if (woodarg == "4b") {
        // wood4 back (V floating balls... not too bad for a cutoff brick)
        firstClick.x = 1.13723; firstClick.y = 0.0923233; firstClick.z = 1.0982;
        secondClick.x = 1.19136; secondClick.y = 0.0328361; secondClick.z = 1.1192;
    } else if (woodarg == "5") {
        // wood5 (V floating balls... else not too bad)
        firstClick.x = 0.761861; firstClick.y = 0.0251733; firstClick.z = 0.450738;
        secondClick.x = 0.801101; secondClick.y = 0.0386602; secondClick.z = 0.530103;
    } else if (woodarg == "6a") {
        // wood6 first (ransac error on thick long face about 9deg)
        firstClick.x = 0.954187; firstClick.y = 0.33911; firstClick.z = 1.01207;
        secondClick.x = 1.01243; secondClick.y = 0.323617; secondClick.z = 1.07247;
    } else if (woodarg == "6b") {
        // wood6 second (offset out of the box... due to ransac error on small face)
        firstClick.x = 0.735326; firstClick.y = -0.000150979; firstClick.z = 0.991789;
        secondClick.x = 0.878452; secondClick.y = 0.00546089; secondClick.z = 1.05055;
    } else if (woodarg == "7a") {
        // wood7 first (V good)
        firstClick.x = 0.935892; firstClick.y = 0.243369; firstClick.z = 1.06142;
        secondClick.x = 1.00011; secondClick.y = 0.376395; secondClick.z = 1.02944;
    } else if (woodarg == "7b") {
        // wood7 second (ransac error on long face)
        firstClick.x = 0.949445; firstClick.y = 0.0588382; firstClick.z = 0.894571;
        secondClick.x = 1.00033; secondClick.y = -0.0704458; secondClick.z = 0.944553;
    } else if (woodarg == "8a") {
        // wood8 top wood (slightly long on top edge)
        firstClick.x = 0.822994; firstClick.y = 0.0164728; firstClick.z = 0.200193;
        secondClick.x = 0.8387120; secondClick.y = 0.103996; secondClick.z = 0.229149;
    } else if (woodarg == "8b") {
        // wood8 bottom wood (flat cube)
        firstClick.x = 0.901611; firstClick.y = -0.0989408; firstClick.z = 0.0616762;
        secondClick.x = 0.903949; secondClick.y = -0.188279; secondClick.z = 0.111148;
    } else if (woodarg == "9a") {
        // wood9 upwood (V really good!)
        firstClick.x = 1.19216; firstClick.y = 0.259848; firstClick.z = 0.280608;
        secondClick.x = 1.13937; secondClick.y = 0.238201; secondClick.z = 0.223305;
    } else if (woodarg == "9b") {
        // wood9 leftwood (V not too bad, changed ec to 4cm)
        firstClick.x = 0.917183; firstClick.y = 0.319945; firstClick.z = 0.146606;
        secondClick.x = 0.742775; secondClick.y = 0.314624; secondClick.z = 0.09697;
    } else if (woodarg == "d1") {
        // debris first wood (thin face hard to fit)
        firstClick.x = 0.474817; firstClick.y = -1.24424; firstClick.z = -0.560593;
        secondClick.x = 0.3045; secondClick.y = -1.2903; secondClick.z = -0.66878;
    } else if (woodarg == "d2") {
        // debris second wood (thin face hard to fit)
        firstClick.x = 0.41927; firstClick.y = -1.38885; firstClick.z = -0.190667;
        secondClick.x = 0.426837; secondClick.y = -1.30189; secondClick.z = -0.333021;
    } else if (woodarg == "d3") {
        // debris third wood
        firstClick.x = 0.781022; firstClick.y = -1.30285; firstClick.z = -0.480864;
        secondClick.x = 0.786624;secondClick.y = -1.40224; secondClick.z = -0.48517;
    } else if (woodarg == "d4") {
        // debris fourth wood
        firstClick.x = 0.702531; firstClick.y = -1.59113; firstClick.z = -0.503632;
        secondClick.x = 0.715156; secondClick.y = -1.48735; secondClick.z = -0.562133;
    }

    //const PointT firstClick(0.333375,-0.000000,0.044450); // model
    //const PointT secondClick(0.301625,-0.035875,0.101600);
    //const PointT firstClick(0.9547, 0.622662, 0.9887); // wood1 (V not bad after change click, shrunken dims)
    //const PointT secondClick(1.0003, 0.621751, 1.05196);
    //const PointT firstClick(0.950628, 0.445224, 0.0538721); // wood2 (V not bad after change click)
    //const PointT secondClick(0.988609, 0.457071, 0.102262);
    //const PointT firstClick(0.967892, 0.534155, 0.974875); // wood3 short (V good except shrunken dims)
    //const PointT secondClick(0.978867, 0.526969, 1.04312);
    //const PointT firstClick(1.11743, -0.0891154, 1.04574); // wood3 long (V good)
    //const PointT secondClick(1.14564, -0.12534, 0.9848);
    //const PointT firstClick(0.911129, -0.0618588, 1.05675); // wood4 front (V not bad after change click)
    //const PointT secondClick(0.992603, 0.0514742, 1.02301);
    //const PointT firstClick(1.13723, 0.0923233, 1.0982); // wood4 back (V floating balls... not too bad for a cutoff brick)
    //const PointT secondClick(1.19136, 0.0328361, 1.1192);
    //const PointT firstClick(0.761861, 0.0251733, 0.450738); // wood5 (V floating balls... else not too bad)
    //const PointT secondClick(0.801101, 0.0386602, 0.530103);
    //const PointT firstClick(0.954187, 0.33911, 1.01207); // wood6 first (ransac error on thick long face about 9deg)
    //const PointT secondClick(1.01243, 0.323617, 1.07247);
    //const PointT firstClick(0.735326, -0.000150979, 0.991789); // wood6 second (offset out of the box... due to ransac error on small face)
    //const PointT secondClick(0.878452, 0.00546089, 1.05055);
    //const PointT firstClick(0.935892, 0.243369, 1.06142); // wood7 first (V good)
    //const PointT secondClick(1.00011, 0.376395, 1.02944);
    //const PointT firstClick(0.949445, 0.0588382, 0.894571); // wood7 second (ransac error on long face)
    //const PointT secondClick(1.00033, -0.0704458, 0.944553);
    //const PointT firstClick(0.822994, 0.0164728, 0.200193); // wood8 top wood (slightly long on top edge)
    //const PointT secondClick(0.8387120, 0.103996, 0.229149);
    //const PointT firstClick(0.901611, -0.0989408, 0.0616762); // wood8 bottom wood (flat cube)
    //const PointT secondClick(0.903949, -0.188279, 0.111148);
    //const PointT firstClick(1.19216, 0.259848, 0.280608); // wood9 upwood (V really good!)
    //const PointT secondClick(1.13937, 0.238201, 0.223305);
    //const PointT firstClick(0.917183, 0.319945, 0.146606); // wood9 leftwood (V not too bad, changed ec to 4cm)
    //const PointT secondClick(0.742775, 0.314624, 0.09697);

    //const PointT firstClick(0.474817, -1.24424, -0.560593); // debris first wood (thin face hard to fit)
    //const PointT secondClick(0.545369, -1.20642, -0.593935);
    //const PointT firstClick(0.440816, -1.39643, -0.224462); // debris second wood (thin face hard to fit)
    //const PointT secondClick(0.380055, -1.36555, -0.21727);
    //const PointT firstClick(0.794728, -1.39358, -0.430554); // debris third wood
    //const PointT secondClick(0.874922, -1.27762, -0.300042);
    //const PointT firstClick(0.702531, -1.59113, -0.503632); // debris fourth wood
    //const PointT secondClick(0.715156, -1.53735, -0.562133);

    /** Extract the K nearest point around the clicked point */
    std::vector <float> kDistances;
    std::vector <int> kIndices;
    pcl::IndicesPtr kIndicesPtr (new std::vector<int> (kIndices));
    
    kDistances.resize(100);
    kIndicesPtr->resize(100);
    
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    kdtree.nearestKSearch (firstClick, 100, *kIndicesPtr, kDistances);

    const PointCloudT cloud_in_const = *cloud_in;
    PointCloudT::Ptr nearestNeighbourPoints ( new PointCloudT(cloud_in_const,*kIndicesPtr));
    
    /** Do ransac with the 100 nearest points of the click point to get the first plane */
    pcl::SACSegmentation<PointT> segPlane;
    pcl::PointIndices::Ptr first_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_first_ptr(new pcl::ModelCoefficients);
    segPlane.setOptimizeCoefficients(true);
    segPlane.setModelType(pcl::SACMODEL_PLANE);//change model to plane
    segPlane.setMethodType(pcl::SAC_RANSAC);
    segPlane.setMaxIterations(100); // Number of iterations
    segPlane.setDistanceThreshold(0.015);

    segPlane.setInputCloud(nearestNeighbourPoints);

    segPlane.segment(*first_plane, *coefficients_first_ptr);

    Eigen::VectorXf coefficients_first;
    coefficients_first.resize(4);
    coefficients_first[0] = coefficients_first_ptr->values[0];
    coefficients_first[1] = coefficients_first_ptr->values[1];
    coefficients_first[2] = coefficients_first_ptr->values[2];
    coefficients_first[3] = coefficients_first_ptr->values[3];

    // Flip normal vector if dot(origin2firstclick, firstplanenormal) > 0 to have normal towards origin 
    if (coefficients_first[0]*firstClick.x + coefficients_first[1]*firstClick.y + coefficients_first[2]*firstClick.z > 0) {
        //std::cout << "Before flip: " << coefficients_first[0] << " " << coefficients_first[1] << " " << coefficients_first[2] << " " << coefficients_first[2] << std::endl;
        coefficients_first[0] = -coefficients_first[0];
        coefficients_first[1] = -coefficients_first[1];
        coefficients_first[2] = -coefficients_first[2];
        coefficients_first[3] = -coefficients_first[3];
        //std::cout << "After flip: " << coefficients_first[0] << " " << coefficients_first[1] << " " << coefficients_first[2] << " " << coefficients_first[2] << std::endl;
    }
   
    /** Get all inliers in the point cloud */
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlaneSeg (new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
    segPlaneSeg->setInputCloud(cloud_in);
    std::vector <int> firstIndices;
    pcl::IndicesPtr firstIndicesPtr (new std::vector<int> (firstIndices));
    segPlaneSeg->selectWithinDistance(coefficients_first,0.015,*firstIndicesPtr);
    PointCloudT::Ptr firstPlane (new PointCloudT(*cloud_in, *firstIndicesPtr)); // Point cloud pointer to all first plane inliers

    pcl::ModelCoefficients::Ptr first_plane_visual (new pcl::ModelCoefficients);
    first_plane_visual->values.resize(4);
    first_plane_visual->values[0]=coefficients_first[0];
    first_plane_visual->values[1]=coefficients_first[1];
    first_plane_visual->values[2]=coefficients_first[2];
    first_plane_visual->values[3]=coefficients_first[3];

    /** Estimate normals of first plane */
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (firstPlane);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr firstnormals (new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(30);

    // Compute the features
    ne.compute (*firstnormals);

    // Filter normals based on directions
    std::vector<int> firstfilterind;
    float thres = 0.3;
    for (int i = 0; i < firstnormals->size(); i++) {
        if (firstnormals->at(i).normal_x > 1*coefficients_first[0] - thres &&
            firstnormals->at(i).normal_x < 1*coefficients_first[0] + thres &&
            firstnormals->at(i).normal_y > 1*coefficients_first[1] - thres &&
            firstnormals->at(i).normal_y < 1*coefficients_first[1] + thres &&
            firstnormals->at(i).normal_z > 1*coefficients_first[2] - thres &&
            firstnormals->at(i).normal_z < 1*coefficients_first[2] + thres)
        firstfilterind.push_back(i);
        //std::cout << "i: " << i << " " << firstnormals->at(i).normal_x << " " << firstnormals->at(i).normal_y << " " << firstnormals->at(i).normal_z << '\n';
    }
  
    PointCloudT::Ptr firstFilter (new PointCloudT(*firstPlane, firstfilterind)); // Select only points that pass the test
    PointCloudT::Ptr firstFilterSOR (new PointCloudT); // Container for after SOR
  
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (firstFilter);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.4); // Small is better for filtering, but too small will filter the good ones
    sor.filter (*firstFilterSOR);

    /** Print out normal for first plane */
    // Put variable here to avoid scope issue
    pcl::PointCloud<pcl::Normal>::Ptr firstnormals2 (new pcl::PointCloud<pcl::Normal>);
    if (showNormals) {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
        ne2.setInputCloud (firstFilterSOR);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
        ne2.setSearchMethod (tree2);

        // Use all neighbors in a sphere of radius 3cm
        ne2.setKSearch(30);

        // Compute the features
        ne2.compute (*firstnormals2);
    }

    /** Perform Euclidean clustering to get rid of floating point cloud chunks */
    PointCloudT::Ptr firstFilterSOREC (new PointCloudT); // Container for after SOR and EC

    pcl::search::KdTree<PointT>::Ptr treeec (new pcl::search::KdTree<PointT>);
    tree -> setInputCloud(firstFilterSOR);
    tree -> nearestKSearch (firstClick, 1, *kIndicesPtr, kDistances);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.04); // Tolerance in meter(s)... can't be too smaller than pc resolution
    ec.setMinClusterSize (100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(firstFilterSOR);
    ec.extract(cluster_indices);
    
    //printf("click point index %d",(*kIndicesPtr)[0]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    bool is_clicked = false;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if ((*kIndicesPtr)[0]==*pit) is_clicked = true;
            cloud_cluster->points.push_back (firstFilterSOR->points[*pit]);
        }

        cloud_cluster->width = firstFilterSOR->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        if (is_clicked) { 
            firstFilterSOREC = cloud_cluster;
            break;
        }
    }

    /** Measure the second plane */
    kdtree.nearestKSearch (secondClick, 100, *kIndicesPtr, kDistances);
    
    // This point cloud is just for visualization
    PointCloudT::Ptr nearestNeighbourPointsSecond (new PointCloudT(cloud_in_const, *kIndicesPtr));
      
    /** Ransac around k nearest points to get the second plane */
    pcl::SACSegmentation<PointT> segPlane2;
    // pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr second_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_second_ptr(new pcl::ModelCoefficients);
    segPlane2.setOptimizeCoefficients(true);
    segPlane2.setModelType(pcl::SACMODEL_PARALLEL_PLANE); // Change model to parallel plane
    segPlane2.setMethodType(pcl::SAC_RANSAC);
    segPlane2.setMaxIterations(100); // Number of iterations
    segPlane2.setDistanceThreshold(0.015);
    Eigen::Vector3f axis (coefficients_first[0],coefficients_first[1],coefficients_first[2]);
    segPlane2.setAxis(axis); // Set axis for the plane to be parallel to
    segPlane2.setInputCloud(nearestNeighbourPointsSecond);

    segPlane2.segment(*second_plane, *coefficients_second_ptr);

    Eigen::VectorXf coefficients_second;
    coefficients_second.resize(4);
    coefficients_second[0] = coefficients_second_ptr->values[0];
    coefficients_second[1] = coefficients_second_ptr->values[1];
    coefficients_second[2] = coefficients_second_ptr->values[2];
    coefficients_second[3] = coefficients_second_ptr->values[3];

    // Flip normal vector if dot(origin2secondclick, secondplanenormal) > 0 to have normal towards origin
    if (coefficients_second[0]*secondClick.x + coefficients_second[1]*secondClick.y + coefficients_second[2]*secondClick.z > 0) {
        //std::cout << "Before flip: " << coefficients_second[0] << " " << coefficients_second[1] << " " << coefficients_second[2] << " " << coefficients_second[2] << std::endl;
        coefficients_second[0] = -coefficients_second[0];
        coefficients_second[1] = -coefficients_second[1];
        coefficients_second[2] = -coefficients_second[2];
        coefficients_second[3] = -coefficients_second[3];
        //std::cout << "After flip: " << coefficients_second[0] << " " << coefficients_second[1] << " " << coefficients_second[2] << " " << coefficients_second[2] << std::endl;
    }

/* old test code
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlane2 (new pcl::SampleConsensusModelPlane<PointT>(cloud_in, *kIndicesPtr));
    pcl::PointIndices::Ptr second_plane(new pcl::PointIndices);
    PointCloudT::Ptr nearestNeighbourPointsSecond ( new PointCloudT(cloud_in_const,*kIndicesPtr));
    segPlane2->getSamples(iterationNum,samples);
    segPlane2->computeModelCoefficients(samples,coefficients_first);
    std::cout<<coefficients_first<<std::endl;
    //segPlane:project
    //.reset();
*/

    /** Get inliers on the second plane */
    pcl::SampleConsensusModelPlane<PointT>::Ptr segPlaneSeg2 (new pcl::SampleConsensusModelPlane<PointT>(cloud_in));
    segPlaneSeg2->setInputCloud(cloud_in);
    std::vector <int> secondIndices;
    pcl::IndicesPtr secondIndicesPtr (new std::vector<int> (secondIndices));
    segPlaneSeg2->selectWithinDistance(coefficients_second,0.015,*secondIndicesPtr);
    PointCloudT::Ptr secondPlane (new PointCloudT(*cloud_in, *secondIndicesPtr)); // Point cloud pointer to all second plane inliers

    pcl::ModelCoefficients::Ptr second_plane_visual (new pcl::ModelCoefficients);
    second_plane_visual->values.resize(4);
    second_plane_visual->values[0]=coefficients_second[0];
    second_plane_visual->values[1]=coefficients_second[1];
    second_plane_visual->values[2]=coefficients_second[2];
    second_plane_visual->values[3]=coefficients_second[3];

    /** Estimate normals of second plane */
    // Create the normal estimation class, and pass the input dataset to it
    ne.setInputCloud (secondPlane);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr secondtree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (secondtree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr secondnormals (new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(30);

    // Compute the features
    ne.compute (*secondnormals);

    // Filter normals based on directions
    std::vector<int> secondfilterind;
    float secondthres = 0.3;
    for (int i = 0; i < secondnormals->size(); i++) {
      if (secondnormals->at(i).normal_x > 1*coefficients_second[0] - secondthres &&
          secondnormals->at(i).normal_x < 1*coefficients_second[0] + secondthres &&
          secondnormals->at(i).normal_y > 1*coefficients_second[1] - secondthres &&
          secondnormals->at(i).normal_y < 1*coefficients_second[1] + secondthres &&
          secondnormals->at(i).normal_z > 1*coefficients_second[2] - secondthres &&
          secondnormals->at(i).normal_z < 1*coefficients_second[2] + secondthres)
        secondfilterind.push_back(i);
      //std::cout << "i: " << i << " " << secondnormals->at(i).normal_x << " " << secondnormals->at(i).normal_y << " " << secondnormals->at(i).normal_z << '\n';
    }

    // Do a stat outlier removal to get rid of (small) floating points
    PointCloudT::Ptr secondFilter (new PointCloudT(*secondPlane, secondfilterind)); // Select only points that pass the test
    PointCloudT::Ptr secondFilterSOR (new PointCloudT); // Container for after SOR
  
    sor.setInputCloud (secondFilter);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.4); // Small is better for filtering, but too small will filter the good ones
    sor.filter (*secondFilterSOR);

    /** Print out normal for second plane */
    // Put variable here to avoid scope issue
    pcl::PointCloud<pcl::Normal>::Ptr secondnormals2 (new pcl::PointCloud<pcl::Normal>);
    if (showNormals) {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> secondne2;
        secondne2.setInputCloud (secondFilterSOR);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointT>::Ptr secondtree2 (new pcl::search::KdTree<PointT> ());
        secondne2.setSearchMethod (secondtree2);

        // Use all neighbors in a sphere of radius 3cm
        secondne2.setKSearch(30);

        // Compute the features
        secondne2.compute (*secondnormals2);
    }

    /** Perform Euclidean clustering to get rid of floating point cloud chunks */
    PointCloudT::Ptr secondFilterSOREC (new PointCloudT); // Container for after SOR and EC

    std::vector<pcl::PointIndices> cluster_indices2;
    treeec -> setInputCloud(secondFilterSOR);
    treeec -> nearestKSearch (secondClick, 1, *kIndicesPtr, kDistances);
    ec.setClusterTolerance(0.04); // Tolerance in meter(s)... can't be too smaller than pc resolution
    ec.setMinClusterSize (100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(secondFilterSOR);
    ec.extract(cluster_indices2);

    //printf("click point index %d",(*kIndicesPtr)[0]);
    is_clicked = false;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices2.begin(); it != cluster_indices2.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if ((*kIndicesPtr)[0]==*pit) is_clicked = true;
            cloud_cluster->points.push_back (secondFilterSOR->points[*pit]);
        }

        cloud_cluster->width = secondFilterSOR->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        if (is_clicked) { 
            secondFilterSOREC = cloud_cluster;
            break;
        }
    }

/**     the code below is for concave hull boundary extraction.      */
    /** THIS IS *NOT* USED IN CURRRENT ALGORITHM */ 
    /** Project the inliner points on the target plane (increase robustness) */
/*
    PointCloudT::Ptr cloud_projected (new PointCloudT);
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(firstPlane);
    proj.setModelCoefficients(plane_visual);
    proj.filter(*cloud_projected);

*/
    /** search the convex hull on the first surface. */
/*  
    PointCloudT::Ptr cloud_hull (new PointCloudT);
    pcl::ConcaveHull<PointT> chull;
    chull.setAlpha(0.01);
    chull.setInputCloud(cloud_projected);
    chull.reconstruct(*cloud_hull);
  
    std::cout<<"============= cloud_hull =============="<<std::endl;

    std::cout<<cloud_hull->sensor_orientation_.w()<<std::endl;
    std::cout<<cloud_hull->sensor_orientation_.x()<<std::endl;
    std::cout<<cloud_hull->sensor_orientation_.y()<<std::endl;
    std::cout<<cloud_hull->sensor_orientation_.z()<<std::endl;

    std::cout<<"============= cloud_in =============="<<std::endl;

    std::cout<<cloud_in->sensor_orientation_.w()<<std::endl;
    std::cout<<cloud_in->sensor_orientation_.x()<<std::endl;
    std::cout<<cloud_in->sensor_orientation_.y()<<std::endl;
    std::cout<<cloud_in->sensor_orientation_.z()<<std::endl;
*/   

    /** Calculate the intersaction line */
    /** Ref. http://geomalgorithms.com/a05-_intersect-1.html */
    Eigen::Vector3f p1;
    p1[0] = first_plane_visual->values[0];
    p1[1] = first_plane_visual->values[1];
    p1[2] = first_plane_visual->values[2];

    float w1 = first_plane_visual->values[3];
    float a1 = first_plane_visual->values[0];
    float b1 = first_plane_visual->values[1];
    float c1 = first_plane_visual->values[2];

    Eigen::Vector3f p2;
    p2[0] = second_plane_visual->values[0];
    p2[1] = second_plane_visual->values[1];
    p2[2] = second_plane_visual->values[2];
    float w2 = second_plane_visual->values[3];
    float a2 = second_plane_visual->values[0];
    float b2 = second_plane_visual->values[1];
    float c2 = second_plane_visual->values[2];
    
    Eigen::Vector3f line;
    line = p1.cross(p2);
    float absX = (line[0]>=0 ? line[0] : -line[0]);
    float absY = (line[1]>=0 ? line[1] : -line[1]);
    float absZ = (line[2]>=0 ? line[2] : -line[2]);
    float px;
    float py;
    float pz;

    int maxc;
    if (absX >= absY) {
        if (absX >= absZ)
            maxc = 1;
        else
            maxc = 3;
    }
    else {
        if (absY >= absZ)
            maxc = 2;
        else
            maxc = 3;
    }

    switch(maxc) {
        case 1: 
            px = 0;
            pz = -(w1*b2-w2*b1)/(c1*b2-c2*b1);
            py = -(w1*c2-w2*c1)/(b1*c2-b2*c1);
            break;
        case 2:
            py = 0;
            pz = -(w1*a2-w2*a1)/(c1*a2-c2*a1);
            px = -(w1*c2-w2*c1)/(a1*c2-a2*c1);
            break;
        case 3:
            pz = 0;
            py = -(w1*a2-w2*a1)/(b1*a2-b2*a1);
            px = -(w1*b2-w2*b1)/(a1*c2-a2*b1);
    }
    
    pcl::ModelCoefficients::Ptr intersection_line (new pcl::ModelCoefficients);
    intersection_line->values.resize(6);
    intersection_line->values[0] = px;
    intersection_line->values[1] = py;
    intersection_line->values[2] = pz;

    intersection_line->values[3] = line[0];
    intersection_line->values[4] = line[1];
    intersection_line->values[5] = line[2];
    
    /** Project points on the line  */
    PointCloudT::Ptr addedpc(new PointCloudT((*firstFilterSOREC)+(*secondFilterSOREC)));

    pcl::ProjectInliers<PointT> proj_line; 
    proj_line.setModelType(pcl::SACMODEL_LINE);
    proj_line.setInputCloud(addedpc);//cloud_in);
    proj_line.setModelCoefficients(intersection_line);
    PointCloudT::Ptr cloud_projected (new PointCloudT);
    proj_line.filter(*cloud_projected); // Entire input point cloud projected onto this line...

/* yuhan?
    Eigen::Vector4f min_pt1;
    Eigen::Vector4f max_pt1;
    pcl::getMinMax3D(*cloud_projected, min_pt1,max_pt1);
    float distance1  = sqrt((max_pt1[0]-min_pt1[0])*(max_pt1[0]-min_pt1[0])+
                (max_pt1[1]-min_pt1[1])*(max_pt1[1]-min_pt1[1])+
                (max_pt1[2]-min_pt1[2])*(max_pt1[2]-min_pt1[2]));

    proj_line.setInputCloud(secondPlane);
    proj_line.filter(*cloud_projected);
    
    Eigen::Vector4f min_pt2;
    Eigen::Vector4f max_pt2;
    pcl::getMinMax3D(*cloud_projected, min_pt2,max_pt2);
    float distance2  = sqrt((max_pt2[0]-min_pt2[0])*(max_pt2[0]-min_pt2[0])+
                (max_pt2[1]-min_pt2[1])*(max_pt2[1]-min_pt2[1])+
                (max_pt2[2]-min_pt2[2])*(max_pt2[2]-min_pt2[2]));
*/

    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    float distance;

    pcl::getMinMax3D(*cloud_projected, min_pt,max_pt);
    float distance1, distance2, distance3;
    distance1  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));

/* yuhan?
    if (distance1>distance2) {
    distance = distance2;
    min_pt = min_pt2;
    max_pt = max_pt2;
    } 
    else {
    distance = distance1;
    min_pt = min_pt1;
    max_pt = max_pt1;
    } 
*/

    PointT line1Mid;
    line1Mid.x = (max_pt[0]+min_pt[0])/2;
    line1Mid.y = (max_pt[1]+min_pt[1])/2;
    line1Mid.z = (max_pt[2]+min_pt[2])/2;
    
    
    PointT vertex1;
    PointT vertex2;
    float vector_length = sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    vertex1.x = 0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex1.y = 0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex1.z = 0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    vertex2.x = -0.5 * distance1 * line[0] / vector_length + line1Mid.x;
    vertex2.y = -0.5 * distance1 * line[1] / vector_length + line1Mid.y;
    vertex2.z = -0.5 * distance1 * line[2] / vector_length + line1Mid.z;

    /** Calculate the length of the other two edges  */
    pcl::ModelCoefficients::Ptr segPlane3 (new pcl::ModelCoefficients);
    segPlane3->values.resize(4);
    segPlane3->values[0]= line[0];
    segPlane3->values[1]= line[1];
    segPlane3->values[2]= line[2];
    segPlane3->values[3] = -line[0]*vertex1.x-line[1]*vertex1.y-line[2]*vertex1.z;
    
    /** Project the first plane on the third plane */
    PointCloudT::Ptr plane1_projected (new PointCloudT);
    PointCloudT::Ptr edge2_projected (new PointCloudT);

    proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setInputCloud(firstFilterSOREC);
    //proj_line.setModelCoefficients(segPlane3);
    //proj_line.setInputCloud(firstPlane);
    //proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setModelCoefficients(first_plane_visual);
    proj_line.filter(*plane1_projected);
    
    float normalx = line[0]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    float normaly = line[1]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    float normalz = line[2]/sqrt(line[0]*line[0]+line[1]*line[1]+line[2]*line[2]);
    for(size_t i=0; i < plane1_projected->points.size(); ++i) {
        float scale = ((plane1_projected->points[i].x-vertex1.x)*normalx+
        (plane1_projected->points[i].y-vertex1.y)*normaly+
        (plane1_projected->points[i].z-vertex1.z)*normalz);
        edge2_projected->points.push_back (PointT(plane1_projected->points[i].x-scale*normalx,plane1_projected->points[i].y-scale*normaly,plane1_projected->points[i].z-scale*normalz));
    }
    edge2_projected->width = edge2_projected->points.size();
    edge2_projected->height = 1;
    edge2_projected->is_dense = true;

    /** Calculate the length of the other edge of the first plane */
    pcl::getMinMax3D(*edge2_projected, min_pt,max_pt);
    distance2  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));
    
    PointT line2Mid;
    line2Mid.x = (max_pt[0]+min_pt[0])/2;
    line2Mid.y = (max_pt[1]+min_pt[1])/2;
    line2Mid.z = (max_pt[2]+min_pt[2])/2;
    
    /** Project the second plane on the third plane */
    PointCloudT::Ptr plane2_projected (new PointCloudT);
    PointCloudT::Ptr edge3_projected (new PointCloudT);
    
    proj_line.setModelType(pcl::SACMODEL_PLANE);
    proj_line.setInputCloud(secondFilterSOREC);
    proj_line.setModelCoefficients(second_plane_visual);
    proj_line.filter(*plane2_projected);

    for(size_t i=0; i< plane2_projected -> points.size();++i) {
        float scale = ((plane2_projected->points[i].x-vertex1.x)*normalx+
        (plane2_projected->points[i].y-vertex1.y)*normaly+
        (plane2_projected->points[i].z-vertex1.z)*normalz);
        edge3_projected->points.push_back (PointT(plane2_projected->points[i].x-scale*normalx,plane2_projected->points[i].y-scale*normaly,plane2_projected->points[i].z-scale*normalz));
    }
    edge3_projected->width = edge3_projected->points.size ();
    edge3_projected->height = 1;
    edge3_projected->is_dense = true;

    /** Calculate the length of the other edge of the second plane */
    pcl::getMinMax3D(*edge3_projected, min_pt,max_pt);
    distance3  = sqrt((max_pt[0]-min_pt[0])*(max_pt[0]-min_pt[0])+
                (max_pt[1]-min_pt[1])*(max_pt[1]-min_pt[1])+
                (max_pt[2]-min_pt[2])*(max_pt[2]-min_pt[2]));
   
    PointT line3Mid;
    line3Mid.x = (max_pt[0]+min_pt[0])/2;
    line3Mid.y = (max_pt[1]+min_pt[1])/2;
    line3Mid.z = (max_pt[2]+min_pt[2])/2;

    /** Calculate the other six points  */
    PointT vertex3;
    PointT vertex4;
    PointT vertex5;
    PointT vertex6;
    PointT vertex7;
    PointT vertex8;
    
    vertex3.x = 2*(line2Mid.x-vertex1.x)+vertex1.x;
    vertex3.y = 2*(line2Mid.y-vertex1.y)+vertex1.y;
    vertex3.z = 2*(line2Mid.z-vertex1.z)+vertex1.z;

    vertex4.x = 2*(line2Mid.x-vertex1.x)+vertex2.x;
    vertex4.y = 2*(line2Mid.y-vertex1.y)+vertex2.y;
    vertex4.z = 2*(line2Mid.z-vertex1.z)+vertex2.z;
    
    //vertex5.x = line3Mid.x;
    //vertex5.y = line3Mid.y;
    //vertex5.z = line3Mid.z;
   
    vertex5.x = 2*(line3Mid.x-vertex1.x)+vertex1.x;
    vertex5.y = 2*(line3Mid.y-vertex1.y)+vertex1.y;
    vertex5.z = 2*(line3Mid.z-vertex1.z)+vertex1.z;
       
    vertex6.x = 2*(line3Mid.x-vertex1.x)+vertex2.x;
    vertex6.y = 2*(line3Mid.y-vertex1.y)+vertex2.y;
    vertex6.z = 2*(line3Mid.z-vertex1.z)+vertex2.z;
    
    vertex7.x = 2*(line3Mid.x-vertex1.x)+vertex3.x;
    vertex7.y = 2*(line3Mid.y-vertex1.y)+vertex3.y;
    vertex7.z = 2*(line3Mid.z-vertex1.z)+vertex3.z;
    
    vertex8.x = 2*(line3Mid.x-vertex1.x)+vertex4.x;
    vertex8.y = 2*(line3Mid.y-vertex1.y)+vertex4.y;
    vertex8.z = 2*(line3Mid.z-vertex1.z)+vertex4.z;

    /** Printing results */
    tend_ms = time_ms();
    tend_us = time_us();

    std::cout << "Filtering normal for first plane... ";
    std::cout << "\nA total of: " << firstFilterSOR->size() << '\n';
    std::cout << "Clustering first plane... ";
    std::cout << "\nA total of: " << firstFilterSOREC->size() << '\n';

    std::cout << "Filtering normal for second plane... ";
    std::cout << "\nA total of: " << secondFilterSOR->size() << '\n';
    std::cout << "Clustering second plane... ";
    std::cout << "\nA total of: " << secondFilterSOREC->size() << '\n';

    std::cout << "====================distance of the first edge=================" << std::endl;
    std::cout << distance1 << std::endl;
    std::cout << "====================distance of the second edge=================" << std::endl;
    std::cout << distance2 << std::endl;
    std::cout << "====================distance of the third edge=================" << std::endl;
    std::cout << distance3 << std::endl;

    std::cout << "Time start(ms): " << tstart_ms << "   Time end(ms): " << tend_ms << std::endl;
    std::cout << "Time elapsed(ms): " << tend_ms-tstart_ms << std::endl;
    std::cout << "Time start(us): " << tstart_us << "   Time end(us): " << tend_us << std::endl;
    std::cout << "Time elapsed(us): " << tend_us-tstart_us << std::endl;
   
    /** Visualization */
    cloud_in->sensor_orientation_.w()=0;
    cloud_in->sensor_orientation_.x()=0;
    cloud_in->sensor_orientation_.y()=0;
    cloud_in->sensor_orientation_.z()=0;

    nearestNeighbourPoints -> sensor_orientation_ = cloud_in -> sensor_orientation_;
    nearestNeighbourPointsSecond -> sensor_orientation_ = cloud_in -> sensor_orientation_;

    firstPlane -> sensor_orientation_ = cloud_in -> sensor_orientation_;
    secondPlane -> sensor_orientation_ = cloud_in -> sensor_orientation_;
    edge2_projected -> sensor_orientation_ = cloud_in -> sensor_orientation_;
    
    //std::cout<<cloud_in -> sensor_origin_<<std::endl;

    //cloud_hull -> sensor_orientation_ = cloud_in -> sensor_orientation_;
    pcl::visualization::PCLVisualizer viewer("cube");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(nearestNeighbourPoints, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> orange(firstPlane, 255, 150, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(firstPlane, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> violet(secondPlane, 0, 150, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> grey(cloud_in, 200, 200, 200);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(edge2_projected, 0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> black(cloud_hull, 0, 0, 255);
    int v1 = 0;

    viewer.addPointCloud(cloud_in,grey, "pcin", v1); // Input point cloud

    viewer.addPointCloud(firstFilterSOREC,orange, "p1pt", v1); // Final first plane points
    viewer.addPointCloud(secondFilterSOREC,blue, "p2pt", v1); // Final second plane points
    //viewer.addPointCloud(firstPlane, orange, "plane1", v1);
    //viewer.addPointCloud(secondPlane, violet, "plane2", v1);

    viewer.addSphere(firstClick, 0.01, "c1"); // First click location
    viewer.addSphere(secondClick, 0.01, "c2"); // Second click location

    viewer.addPointCloud(nearestNeighbourPoints, red, "knn1", v1); // knn first click
    viewer.addPointCloud(nearestNeighbourPointsSecond, red, "knn2", v1); // knn second click

    viewer.addPointCloud(edge2_projected, green, "edge2", v1); // Edge 2
    viewer.addPointCloud(edge3_projected, green, "edge3", v1); // Edge 3

    //viewer.addPointCloud(cloud_projected,orange, "1st");
    //viewer.addPointCloud(cloud_hull, black, "HULL", v1);

    if (showPlanes) {
        viewer.addPlane(*first_plane_visual, vertex1.x, vertex1.y, vertex1.z,"p1"); // First plane
        viewer.addPlane(*second_plane_visual, vertex1.x, vertex1.y, vertex1.z,"p2"); // Second plane
        viewer.addPlane(*segPlane3, vertex1.x, vertex1.y, vertex1.z,"p3"); // Third plane
    }

    viewer.addLine(*intersection_line); // Intersection line

    viewer.addSphere(vertex1, 0.02, "v1"); // Vertices
    viewer.addSphere(vertex2, 0.02, "v2");
    viewer.addSphere(vertex3, 0.02, "v3");
    viewer.addSphere(vertex4, 0.02, "v4");
    viewer.addSphere(vertex5, 0.02, "v5");
    viewer.addSphere(vertex6, 0.02, "v6");
    viewer.addSphere(vertex7, 0.02, "v7");
    viewer.addSphere(vertex8, 0.02, "v8");

    viewer.setBackgroundColor(0, 0, 0);//255,255,255);

    if (showNormals) {
        viewer.addPointCloudNormals<PointT, pcl::Normal> (firstFilterSOR, firstnormals2, 1, 0.05, "n1"); // Normals for first plane
        viewer.addPointCloudNormals<PointT, pcl::Normal> (secondFilterSOR, secondnormals2, 1, 0.05, "n2"); // Normals for second plane
    }

    viewer.spin(); // Start viewer
}


